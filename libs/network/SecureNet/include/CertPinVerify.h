#pragma once

// FreeInk SDK — the wolfSSL side of CertPin.h. Header-only and wolfSSL-only, so a
// host test can drive real handshakes through it.

#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/sha256.h>

#include <cstring>

#include "CertPin.h"

namespace freeink {
namespace cert_pin {

// wolfSSL verify callback. Only called with preverify == 0 when the chain or the
// host check failed; the pin is the last word then.
inline int verify(int preverify, WOLFSSL_X509_STORE_CTX* store) {
  if (preverify) return 1;
  auto* state = store ? static_cast<State*>(store->userCtx) : nullptr;
  if (!state || store->totalCerts < 1 || !store->certs) return 0;
  uint8_t leaf[SHA256_SIZE];
  if (wc_Sha256Hash(store->certs[0].buffer, store->certs[0].length, leaf) != 0) return 0;
  if (state->hasPin && std::memcmp(leaf, state->pin, SHA256_SIZE) == 0) return 1;
  state->rejected = true;
  std::memcpy(state->seen, leaf, SHA256_SIZE);
  return 0;
}

// Verify `ssl`'s peer against its CTX's CAs, require the leaf to name `host`, and
// let `state`'s pin vouch for a peer that fails either. `state` must outlive the
// handshake.
inline void arm(WOLFSSL* ssl, const char* host, State* state) {
  state->rejected = false;
  wolfSSL_set_verify(ssl, WOLFSSL_VERIFY_PEER, verify);
  wolfSSL_SetCertCbCtx(ssl, state);
  wolfSSL_check_domain_name(ssl, host);
}

}  // namespace cert_pin
}  // namespace freeink
