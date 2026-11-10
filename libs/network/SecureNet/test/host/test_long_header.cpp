// A GitHub release redirect carries a Content-Security-Policy line over MAX_LINE.
// The response must still parse: Location and Content-Length survive it.
#include <SecureHttpClient.h>

#include <cassert>
#include <cstdio>

namespace freeink {
SecureClient::~SecureClient() = default;
void SecureClient::setCACert(const char*) {}
void SecureClient::setInsecure() {}
void SecureClient::setPinnedCert(const uint8_t*) {}
int SecureClient::connect(IPAddress, uint16_t) { return 0; }
int SecureClient::connect(const char*, uint16_t) { return 0; }
size_t SecureClient::write(uint8_t) { return 0; }
size_t SecureClient::write(const uint8_t*, size_t) { return 0; }
int SecureClient::available() { return 0; }
int SecureClient::read() { return -1; }
int SecureClient::read(uint8_t*, size_t) { return -1; }
int SecureClient::peek() { return -1; }
void SecureClient::flush() {}
void SecureClient::stop() {}
uint8_t SecureClient::connected() { return 0; }
bool SecureClient::tls13Available() { return false; }
bool hostResolves(const char*) { return true; }
}  // namespace freeink

static int get(const std::string& reply, freeink::SecureHttpClient& http) {
  FakeNet::replies().assign({reply});
  FakeNet::requests().clear();
  assert(http.begin("http://example.test/file"));
  return http.GET([](const uint8_t*, size_t) { return true; });
}

int main() {
  const std::string head = "HTTP/1.1 302 Found\r\nLocation: http://cdn.test/x\r\n";
  const std::string tail = "Content-Length: 0\r\n\r\n";
  {
    freeink::SecureHttpClient http;
    assert(get(head + "Content-Security-Policy: " + std::string(4150, 'a') + "\r\n" + tail, http) == 302);
    assert(http.getHeader("location") == "http://cdn.test/x");
    assert(http.hasContentLength() && http.responseComplete());
  }
  {
    // Any other oversized header still fails the request.
    freeink::SecureHttpClient http;
    assert(get(head + "X-Other: " + std::string(4150, 'a') + "\r\n" + tail, http) == -1);
  }
  {
    // The total header budget holds even for the skipped header.
    freeink::SecureHttpClient http;
    assert(get(head + "Content-Security-Policy: " + std::string(9000, 'a') + "\r\n" + tail, http) == -1);
  }
  puts("test_long_header: ok");
  return 0;
}
