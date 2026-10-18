#pragma once

// FreeInk SDK — certificate pinning on top of CA verification.
//
// A peer is trusted when its chain verifies against the CAs loaded in the CTX and
// its leaf names the host, as usual. When it does not (a self-signed home server, a
// private CA), a pin can still vouch for it: the SHA-256 of the exact leaf
// certificate the user chose to trust. A rejected peer leaves its leaf's SHA-256
// behind, so the caller can show it and ask.
//
// This header is the state only, free of wolfSSL; CertPinVerify.h has the callback.

#include <cstddef>
#include <cstdint>

namespace freeink {
namespace cert_pin {

constexpr size_t SHA256_SIZE = 32;

struct State {
  bool hasPin = false;
  uint8_t pin[SHA256_SIZE] = {};
  // Set by a handshake that failed verification: the leaf the peer presented.
  bool rejected = false;
  uint8_t seen[SHA256_SIZE] = {};
};

}  // namespace cert_pin
}  // namespace freeink
