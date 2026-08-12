# Note for Tianxing — Python `StreamPublisher.push` is unusable without two helpers

**From:** miniMUAS v2 live-video integration (predictive stream → dashboard)
**Build:** NDNSF `main` @ `75d3f5e` (streaming API), pybind wrapper, ndn-cxx 0.9.0
**TL;DR:** From Python there is no way to construct a Data that
`LiveStreamPublisher::publishSignedData` will accept, so the high-level
`StreamPublisher.push` cannot be driven from the Python wrapper at all. Two small
bindings fix it; we've implemented them locally as `stream-push-bindings.patch`
and would love them upstream.

## What the Core requires
`ndn-service-framework/Stream.cpp` `LiveStreamPublisher::publishSignedData`:

```cpp
const auto root = m_definition.mappingRoot();
if (!root.isPrefixOf(name) || name.size() != root.size() + 3 ||
    name[root.size()].toUri() != "v" ||
    !name[root.size() + 1].isNumber() ||
    name[root.size() + 1].toNumber() != m_definition.mappingVersion ||
    !name[root.size() + 2].isSequenceNumber()) {
  throw std::invalid_argument("non-canonical predictive Data name");
}
```

So a pushed frame's name must be **exactly** `mappingRoot + ["v",
Number(mappingVersion), SequenceNumber(seq)]` — the same thing
`nsf::makePredictiveDataName(definition, sequence)` builds, and the same thing the
C++ example does before `stream->push(...)`:

```cpp
// examples/StreamFacadeProvider.cpp
auto data = /* Data named */ nsf::makePredictiveDataName(descriptor.definition, sequence);
... sign ...
stream->push(std::move(data));
```

## Why Python can't do this today
1. **`makePredictiveDataName` is not exposed** in `pythonWrapper`. A Python app
   can't get the canonical name. Reconstructing it by hand is a trap: the version
   is `appendNumber(mappingVersion)` — a nonNegativeInteger component (byte
   `0x03` for version 3), **not** the ASCII string `"3"` — so an f-string name
   fails `isNumber()`/the value check.
2. **No exact-name signer is exposed.** The only signer in the wrapper,
   `make_segmented_data_packets`, does:
   ```cpp
   ndn::Name versionedName(baseName);
   versionedName.appendVersion(...);        // <-- appends a version component
   ndn::Segmenter segmenter(...); ...        // <-- and a segment component
   ```
   so the resulting name is `baseName/<version>/<segment>` — it can never equal
   `root + 3`. Even with the canonical name string, there's no way to sign a Data
   that keeps the name verbatim.

Net: `push()` is exported, but nothing in the wrapper can produce a wire it will
accept. (Reproduced end-to-end: a full controller+provider+consumer stack
bootstraps fine — RSA identities, ABE policy, tokens-on cert bootstrap all work —
the stream session opens, and the *first* `push` throws "non-canonical predictive
Data name".)

## Proposed fix (what we implemented locally)
Two free-function bindings in `_ndnsf.cpp` + thin Python wrappers:

```cpp
// exact-name signer: make_segmented_data_packets MINUS appendVersion/segmentation
py::bytes makeSignedData(const std::string& name, const py::bytes& content,
                         const std::string& signingIdentity, int freshnessMs) {
  ndn::KeyChain keyChain;
  const auto id = signingIdentity.empty() ? ndn::Name("/ndnsf/python/signed-data")
                                          : ndn::Name(signingIdentity);
  getOrCreateIdentity(keyChain, id);
  ndn::Data data(ndn::Name{name});
  data.setFreshnessPeriod(ndn::time::milliseconds(freshnessMs));
  const std::string b = content;
  data.setContent(ndn::span<const uint8_t>(
      reinterpret_cast<const uint8_t*>(b.data()), b.size()));
  keyChain.sign(data, ndn::security::SigningInfo(
      ndn::security::SigningInfo::SIGNER_TYPE_ID, id));
  const auto w = data.wireEncode();
  return py::bytes(reinterpret_cast<const char*>(w.wire()), w.size());
}

// canonical predictive name (URI) — same 3 lines as makePredictiveDataName
std::string makePredictiveDataNameUri(const std::string& mappingRoot,
                                      uint64_t mappingVersion, uint64_t sequence) {
  ndn::Name name(mappingRoot);
  name.append("v").appendNumber(mappingVersion);
  name.appendSequenceNumber(sequence);
  return name.toUri();
}
```
Exposed as `ndnsf.make_signed_data(name, content, signing_identity="",
freshness_ms=300)` and `ndnsf.make_predictive_data_name(mapping_root,
mapping_version, sequence)`.

**Nicer still if you prefer:** expose `makePredictiveDataName(definition, sequence)`
directly (taking the wrapper `LiveStreamDefinition`), and/or a one-shot
`make_predictive_stream_data(definition, sequence, content, signing_identity,
freshness_ms) -> bytes` that names+signs in one call — that's exactly the app's
need and avoids the name→URI→name round-trip.

## Usage after the fix (miniMUAS `video_stream.py`)
```python
name = ndnsf.make_predictive_data_name(defn.mapping_root, defn.mapping_version, seq)
wire = ndnsf.make_signed_data(name, jpeg, signing_identity=identity, freshness_ms=300)
stream.push(wire)
```

Thanks! The streaming API is otherwise a great fit for our live drone video →
dashboard path (fixes the bursty framerate + producer-churn stall we had on the
per-frame segmented path). These two helpers are the only thing between us and a
field A/B of it.
