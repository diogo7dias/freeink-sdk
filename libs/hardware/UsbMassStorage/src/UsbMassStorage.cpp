#include "UsbMassStorage.h"

#if FREEINK_CAP_USB_MSC

#if !__has_include(<tinyusb.h>)
#error \
    "FREEINK_CAP_USB_MSC needs espressif/esp_tinyusb: add the component (with CONFIG_TINYUSB_MSC_ENABLED) and put its include dirs and libs on the build"
#endif

#include <Arduino.h>
#include <esp_pm.h>
#include <tinyusb.h>
#include <tinyusb_default_config.h>
#include <tusb.h>

#include <algorithm>
#include <atomic>
#include <cstring>

// Built on esp_tinyusb (the IDF component) rather than Arduino's USBMSC, so it also
// works in builds that rebuild the Arduino core without its TinyUSB layer. The SCSI
// callbacks below are TinyUSB's own. esp_tinyusb's MSC glue (tinyusb_msc.c, a FATFS
// storage layer) stays out of the link because the two hooks its driver core calls are
// defined here; were it linked anyway, its tud_msc_* would clash with these at link time.
extern "C" void msc_storage_mount_to_usb(void) {}
extern "C" void msc_storage_mount_to_app(void) {}

namespace freeink {
namespace {

std::atomic<UsbMassStorage*> gOwner{nullptr};
std::atomic<FsBlockDeviceInterface*> gDev{nullptr};
constexpr uint16_t kBlockSize = 512;
uint8_t gSectorScratch[kBlockSize];

bool isRangeValid(FsBlockDeviceInterface* dev, const uint32_t lba, const uint32_t offset, const uint32_t bufsize) {
  if (!dev || bufsize == 0) return false;
  const uint64_t totalBytes = static_cast<uint64_t>(dev->sectorCount()) * kBlockSize;
  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  const uint64_t end = start + bufsize;
  return end >= start && end <= totalBytes;
}

int32_t mscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  auto* const dev = gDev.load();
  auto* const owner = gOwner.load();
  if (!buffer || !isRangeValid(dev, lba, offset, bufsize)) {
    if (owner) owner->markIoError();
    return -1;
  }

  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  auto* output = static_cast<uint8_t*>(buffer);
  uint32_t remaining = bufsize;
  uint64_t cursor = start;

  if ((offset % kBlockSize) == 0 && (bufsize % kBlockSize) == 0) {
    const size_t sectors = bufsize / kBlockSize;
    if (!dev->readSectors(static_cast<Sector_t>(start / kBlockSize), output, sectors)) {
      if (owner) owner->markIoError();
      return -1;
    }
  } else {
    while (remaining > 0) {
      const auto sector = static_cast<Sector_t>(cursor / kBlockSize);
      const size_t within = static_cast<size_t>(cursor % kBlockSize);
      const size_t count = std::min<size_t>(kBlockSize - within, remaining);
      if (!dev->readSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      memcpy(output, gSectorScratch + within, count);
      output += count;
      cursor += count;
      remaining -= count;
    }
  }

  if (owner) owner->markAccessed();
  return static_cast<int32_t>(bufsize);
}

int32_t mscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  auto* const dev = gDev.load();
  auto* const owner = gOwner.load();
  if (!buffer || !isRangeValid(dev, lba, offset, bufsize)) {
    if (owner) owner->markIoError();
    return -1;
  }

  const uint64_t start = static_cast<uint64_t>(lba) * kBlockSize + offset;
  auto* input = buffer;
  uint32_t remaining = bufsize;
  uint64_t cursor = start;

  if ((offset % kBlockSize) == 0 && (bufsize % kBlockSize) == 0) {
    const size_t sectors = bufsize / kBlockSize;
    if (!dev->writeSectors(static_cast<Sector_t>(start / kBlockSize), input, sectors)) {
      if (owner) owner->markIoError();
      return -1;
    }
  } else {
    while (remaining > 0) {
      const auto sector = static_cast<Sector_t>(cursor / kBlockSize);
      const size_t within = static_cast<size_t>(cursor % kBlockSize);
      const size_t count = std::min<size_t>(kBlockSize - within, remaining);
      if (!dev->readSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      memcpy(gSectorScratch + within, input, count);
      if (!dev->writeSector(sector, gSectorScratch)) {
        if (owner) owner->markIoError();
        return -1;
      }
      input += count;
      cursor += count;
      remaining -= count;
    }
  }

  if (owner) owner->markAccessed();
  return static_cast<int32_t>(bufsize);
}

// USB Drive keeps the chip out of automatic light sleep: the USB-OTG link does not
// survive it. Created on first use; null when power management is compiled out.
esp_pm_lock_handle_t gNoSleepLock = nullptr;

}  // namespace
}  // namespace freeink

extern "C" {

void tud_msc_inquiry_cb(uint8_t, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
  memcpy(vendor_id, "FreeInk ", 8);
  memcpy(product_id, "SD Card         ", 16);
  memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
  if (freeink::gDev.load()) return true;
  tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);  // medium not present
  return false;
}

void tud_msc_capacity_cb(uint8_t, uint32_t* block_count, uint16_t* block_size) {
  auto* const dev = freeink::gDev.load();
  *block_count = dev ? dev->sectorCount() : 0;
  *block_size = freeink::kBlockSize;
}

bool tud_msc_start_stop_cb(uint8_t, uint8_t, bool start, bool load_eject) {
  if (load_eject && !start) {
    if (auto* const owner = freeink::gOwner.load()) owner->markEjected();
  }
  return true;
}

int32_t tud_msc_read10_cb(uint8_t, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  return freeink::mscRead(lba, offset, buffer, bufsize);
}

int32_t tud_msc_write10_cb(uint8_t, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  return freeink::mscWrite(lba, offset, buffer, bufsize);
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const /*scsi_cmd*/[16], void*, uint16_t) {
  // TinyUSB answers the commands a host needs (including PREVENT ALLOW MEDIUM
  // REMOVAL) itself; only the rest arrive here.
  tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);  // invalid command operation code
  return -1;
}

}  // extern "C"

namespace freeink {

bool UsbMassStorage::begin(FsBlockDeviceInterface* dev) {
  if (_active || !dev || dev->sectorCount() == 0) return false;

  if (!gNoSleepLock) {
    const esp_err_t err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "usb_msc", &gNoSleepLock);
    if (err != ESP_OK) {
      gNoSleepLock = nullptr;
      if (err != ESP_ERR_NOT_SUPPORTED && Serial) {
        Serial.printf("[%lu] [USB] no-light-sleep lock failed: %s\n", millis(), esp_err_to_name(err));
      }
    }
  }
  if (gNoSleepLock) esp_pm_lock_acquire(gNoSleepLock);

  gDev.store(dev);
  gOwner.store(this);
  _state.store(UsbMassStorageState::WaitingForHost);
  _hostSeen.store(false);
  const tinyusb_config_t config = TINYUSB_DEFAULT_CONFIG();
  if (tinyusb_driver_install(&config) != ESP_OK) {
    gOwner.store(nullptr);
    gDev.store(nullptr);
    _state.store(UsbMassStorageState::Idle);
    if (gNoSleepLock) esp_pm_lock_release(gNoSleepLock);
    return false;
  }
  _active = true;
  return true;
}

void UsbMassStorage::end() {
  if (!_active) return;
  const esp_err_t err = tinyusb_driver_uninstall();
  if (err != ESP_OK && Serial)
    Serial.printf("[%lu] [USB] TinyUSB uninstall failed: %s\n", millis(), esp_err_to_name(err));
  if (gNoSleepLock) esp_pm_lock_release(gNoSleepLock);
  gOwner.store(nullptr);
  gDev.store(nullptr);
  _active = false;
  _state.store(UsbMassStorageState::Idle);
  _hostSeen.store(false);
}

UsbMassStorageState UsbMassStorage::state() const {
  if (!_active) return UsbMassStorageState::Idle;
  const auto current = _state.load();
  if (current == UsbMassStorageState::Ejected) return current;

  if (!tud_mounted()) {
    return _hostSeen.load() ? UsbMassStorageState::Disconnected : UsbMassStorageState::WaitingForHost;
  }

  // Keep the error visible while the host is mounted, but report a later cable
  // removal so the application can safely reclaim its raw storage session.
  if (current == UsbMassStorageState::IoError) return current;

  _hostSeen.store(true);
  auto expected = UsbMassStorageState::WaitingForHost;
  _state.compare_exchange_strong(expected, UsbMassStorageState::Connected);
  return _state.load();
}

bool UsbMassStorage::hostConnected() const {
  const auto current = state();
  return current == UsbMassStorageState::Connected || current == UsbMassStorageState::Accessed;
}

bool UsbMassStorage::disconnectHost() const { return _active && tud_disconnect(); }

void UsbMassStorage::markAccessed() const {
  auto current = _state.load();
  while (current != UsbMassStorageState::Ejected && current != UsbMassStorageState::IoError) {
    if (_state.compare_exchange_weak(current, UsbMassStorageState::Accessed)) return;
  }
}

void UsbMassStorage::markEjected() const { _state.store(UsbMassStorageState::Ejected); }

void UsbMassStorage::markIoError() const {
  _hostSeen.store(true);
  _state.store(UsbMassStorageState::IoError);
}

}  // namespace freeink

#endif  // FREEINK_CAP_USB_MSC
