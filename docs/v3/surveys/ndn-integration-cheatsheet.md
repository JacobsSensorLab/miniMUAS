# NDN stack integration cheat-sheet (drone-fleet agent builders)

Every snippet below is transcribed from real source/tests at the cited absolute paths (verified 2026-07-10 by reading the files; nothing was compiled fresh — all crates have warm `target/` dirs from prior builds). Three repos are involved:

- `ndn-rs` — `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs` (forwarder, faces, app SDK, security)
- `ndn-ext` — `/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext` (service layer, extra faces)
- `ndn-sim` — `/Users/pmle/Documents/Dev/ndn-workspace/ndn-sim` (ndn-lab simulator; crate name `ndn-sim`)
- `ndf-rs` — `/Users/pmle/Documents/Dev/ndf-rs/refounding` (`ndf-apps`, `ndf-spark`, `ndf-core`, `ndf-policy`, `ndf-nfn`)

---

## 1. Engine bring-up (`ndn-engine`)

Crate: `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/forwarding/ndn-engine/`

Key exports (`src/lib.rs`): `EngineBuilder`, `EngineConfig` (from `builder`, **not on wasm32**; wasm gets `WasmEngineBuilder`), `ForwarderEngine`, `ShutdownHandle`, `FaceState`, `DataPlane`, plus `pub use ndn_runtime::{Runtime, Spawn}`.

### Minimal in-process engine

```rust
use ndn_engine::builder::{EngineBuilder, EngineConfig};

// async — must run inside a tokio runtime. Returns the running engine + shutdown handle.
let (engine, shutdown) = EngineBuilder::new(EngineConfig::default()).build().await?;
// ...
shutdown.shutdown().await;      // or shutdown.detach();
```

Exact signatures (`src/builder.rs`, `src/engine.rs`):

```rust
pub struct EngineBuilder { /* .. */ }
impl EngineBuilder {
    pub fn new(config: EngineConfig) -> Self;
    pub fn face<T: Transport>(mut self, transport: T) -> Self;          // deferred insert
    pub fn face_composed(mut self, face: ndn_transport::Face) -> Self;  // custom LinkService
    pub fn alloc_face_id(&self) -> ndn_transport::FaceId;               // pre-allocate before build
    pub fn strategy<S: ErasedStrategy>(mut self, s: S) -> Self;
    pub fn security(mut self, mgr: SecurityManager) -> Self;
    pub fn security_profile(mut self, p: SecurityProfile) -> Self;
    pub fn validator(mut self, v: Arc<Validator>) -> Self;
    pub fn schema_rule(mut self, rule: SchemaRule) -> Self;
    pub fn content_store(mut self, cs: Arc<dyn ErasedContentStore>) -> Self;
    pub fn runtime(mut self, rt: Arc<dyn Runtime>) -> Self;             // default: ndn_runtime::default_runtime()
    pub fn replay_guard_disabled(self) -> Self;                         // test-only escape hatch
    pub fn unsolicited_data_policy(mut self, policy: UnsolicitedDataPolicy) -> Self; // default DropAll; AdmitNetwork for broadcast bearers
    pub async fn build(mut self) -> Result<(ForwarderEngine, ShutdownHandle)>;
}
```

`EngineConfig` defaults (`src/builder.rs:86`): `pipeline_channel_cap: 4096`, `cs_capacity_bytes: 64 MiB`, `pipeline_threads: 0` (auto), `data_plane: DataPlane::default()` (Shared; `Partitioned` needs the `partitioned-fwd` feature), `require_local_validation: false`.

`ForwarderEngine` is `Clone`-able and is the *same type* in production and inside ndn-sim. Handy accessors (`src/engine.rs`): `engine.fib()`, `engine.pit()`, `engine.cs()`, `engine.faces()`, `engine.strategy_table()`, `engine.signals()`, `engine.validator()`, `engine.runtime()`, and:

```rust
pub fn add_face<F: ndn_transport::Transport + 'static>(&self, face: F, cancel: CancellationToken);
pub fn remove_face(&self, face_id: FaceId);
pub async fn inject_packet(/* .. */);
```

`ShutdownHandle`: `cancel_token() -> CancellationToken`, `async shutdown(self)`, `detach(self)`.

### Connecting an app / two engines in-proc: `ndn-face-local`

Crate: `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/faces/ndn-face-local/src/lib.rs` (re-exported as `ndn_face::local::InProcFace` from `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/faces/ndn-face/`).

```rust
pub struct InProcFace { /* engine side, implements Transport */ }
pub struct InProcHandle { /* app side */ }
impl InProcFace {
    pub fn new(id: FaceId, buffer: usize) -> (Self, InProcHandle);   // FaceKind::App
    pub fn new_kind(id: FaceId, buffer: usize, kind: FaceKind) -> (Self, InProcHandle);
}
impl InProcHandle {
    pub fn face_id(&self) -> FaceId;
    pub async fn send(&self, pkt: Bytes) -> Result<(), FaceError>;
    pub async fn recv(&self) -> Option<Bytes>;
    pub async fn recv_tagged(&self) -> Option<TaggedBytes>;  // TaggedBytes { wire, source_face: Option<FaceId> }
}
```

Canonical one-engine, two-face wiring — from `/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/ndn-rpc/tests/face_carrier.rs`:

```rust
let (c_face, c_handle) = InProcFace::new(FaceId(1), 64);
let (p_face, p_handle) = InProcFace::new(FaceId(2), 64);
let (engine, shutdown) = EngineBuilder::new(EngineConfig::default())
    .face(c_face)
    .face(p_face)
    .build().await.unwrap();
engine.fib().add_nexthop(&svc_prefix, FaceId(2), 0);   // route prefix -> producer face
```

Note there is no dedicated "engine-to-engine in-proc pipe" type in ndn-face-local; in-proc multi-engine topologies are ndn-sim's `SimLink` job (§5). The simpler route for two same-process apps is one engine + `engine.app_node(..)` per app (§3 sidebar below).

### Two engines over UDP

`UdpFace` — `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/faces/ndn-face/src/net/udp.rs`:

```rust
impl UdpFace {
    pub async fn bind(local: SocketAddr, peer: SocketAddr, id: FaceId) -> std::io::Result<Self>;
    pub fn from_socket(id: FaceId, socket: UdpSocket, peer: SocketAddr) -> Self;
    pub fn from_shared_socket(id: FaceId, socket: Arc<UdpSocket>, peer: SocketAddr) -> Self;
}
```

Verified two-engine UDP wiring, from `/Users/pmle/Documents/Dev/ndf-rs/refounding/ndf-apps/tests/compute_socket.rs`:

```rust
let addr_a: std::net::SocketAddr = "127.0.0.1:47831".parse().unwrap();
let addr_b: std::net::SocketAddr = "127.0.0.1:47832".parse().unwrap();

let (engine_a, shutdown_a) = EngineBuilder::new(EngineConfig::default()).build().await.expect("engine a");
shutdown_a.detach();
let (engine_b, shutdown_b) = EngineBuilder::new(EngineConfig::default()).build().await.expect("engine b");
shutdown_b.detach();

let face_a = UdpFace::bind(addr_a, addr_b, FaceId(1)).await.expect("bind a");
engine_a.add_face(face_a, CancellationToken::new());
let face_b = UdpFace::bind(addr_b, addr_a, FaceId(1)).await.expect("bind b");
engine_b.add_face(face_b, CancellationToken::new());

// Route the remote namespace out the UDP face:
engine_b.fib().add_nexthop(&"/compute/alice".parse().unwrap(), FaceId(1), 0);
```

(Imports there: `use ndn_face::UdpFace; use ndn_transport::FaceId;`.) FaceIds are per-engine, so both sides can use `FaceId(1)`. `UdpFace` uses an unconnected socket (`send_to`/`recv_from`) deliberately — macOS ICMP-unreachable EPIPE workaround (comment at `udp.rs:26-33`). TCP variant: `ndn_face::net::tcp_face_connect(id, addr)`.

---

## 2. Service layer: `ndn-service-core` + `ndn-service-macro` + `ndn-rpc` + `ndn-ndnsf`

All under `/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/`.

### The seam (`ndn-service-core/src/lib.rs`)

```rust
pub struct ServiceId(Name);           // ServiceId::new(prefix: Name), .name() -> &Name; From<Name>
pub struct OpId(String);              // OpId::new(name), .as_str()

pub trait Frame: Sized + Send {
    fn encode(&self) -> Bytes;
    fn decode(bytes: &[u8]) -> Result<Self, ServiceError>;
}
// Frame is provided for: (), String, Vec<u8>, Bytes, bool, u32, u64, i32, i64.

pub enum ServiceError { Decode(String), NotFound, Handler(String), Transport(String), Unauthorized(String) }

pub struct Response  { pub producer: Name, pub payload: Bytes }
pub struct Invocation { pub op: OpId, pub request: Bytes, pub requester: Option<Name> }

#[async_trait] pub trait Dispatch: Send + Sync + 'static {
    async fn dispatch(&self, invocation: Invocation) -> Result<Bytes, ServiceError>;
}

#[async_trait] pub trait Carrier: Send + Sync {
    async fn invoke(&self, svc: &ServiceId, op: &OpId, request: Bytes) -> Result<Response, ServiceError>;
    async fn serve(&self, svc: &ServiceId, dispatch: Arc<dyn Dispatch>) -> Result<(), ServiceError>;
}

pub enum Strategy { FirstResponding, Random, All }
#[async_trait] pub trait SelectCarrier: Carrier {
    async fn invoke_select(&self, svc: &ServiceId, op: &OpId, request: Bytes, strategy: Strategy)
        -> Result<Vec<Response>, ServiceError>;
}
#[async_trait] pub trait HintedCarrier: Carrier {
    async fn invoke_hinted(&self, svc: &ServiceId, op: &OpId, request: Bytes, hint: Option<&Name>)
        -> Result<Response, ServiceError>;
}
```

Also in core: `ScriptDispatch` (runtime op-keyed `bytes -> bytes` handlers, for Python/FFI front-ends; panic-isolated) and `ndn_service_core::framing::{encode_fields, read_field}` — u32-LE length-prefixed field concatenation; **decode ignores trailing bytes, so appending fields is forward-compatible; reordering/removing is breaking** (doc at `lib.rs:295-301`).

`ndn-service-core` is `no_std + alloc` for the message layer; the async traits need default feature `std`. `src/publish.rs` adds the embedded leaf producer (`Publisher<T>` + `PublicationSink`) for ESP32-class devices — see the runnable doc-test in that file and `examples/embedded_sensor.rs`.

### Defining a service — `#[ndn_service]` (exact syntax)

From `/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/ndn-rpc/tests/macro_service.rs` (real, compiling test):

```rust
use ndn_service_macro::{Frame, ndn_service};

#[derive(Frame, Debug, PartialEq, Eq)]     // #[derive(Frame)] = named-field structs only
struct Stats { sum: u64, even: bool, label: String }

#[ndn_service]
trait Calc {
    async fn add(&self, a: u64, b: u64) -> u64;
    async fn echo(&self, msg: String) -> String;
    async fn ping(&self) -> u64;
    async fn summarize(&self, a: u64, b: u64) -> Stats;
}

struct CalcImpl;
impl Calc for CalcImpl {                    // plain async fns — NO #[async_trait]
    async fn add(&self, a: u64, b: u64) -> u64 { a + b }
    /* ... */
}
```

What the macro generates (`ndn-service-macro/src/lib.rs`):
- Per-op request structs `CalcAddRequest`, `CalcEchoRequest`, … (`{Trait}{PascalOp}Request`), each implementing `Frame` via `framing::encode_fields`.
- `pub struct CalcDispatch<S: Calc>(pub Arc<S>)` implementing `ndn_service_core::Dispatch` — matches `invocation.op.as_str()` to decode-request → call impl → encode return.
- `pub struct CalcClient<C: Carrier> { carrier, svc }` with `CalcClient::new(carrier: C, svc: ServiceId)`, one `pub async fn add(&self, a: u64, b: u64) -> Result<u64, ServiceError>` per op, plus `add_select(a, b, strategy: Strategy) -> Result<Vec<(Name, u64)>, ServiceError>` gated `where C: SelectCarrier`.
- Trait methods are rewritten to `-> impl Future<Output = T> + Send` (RPITIT) and the trait gains `Send + Sync + 'static` supertraits. Args must be plain identifiers; arg/return types must implement `Frame`.

### Hosting + calling over `ndn-rpc`

`ndn-rpc` (`/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/ndn-rpc/`) has two carriers:

**`RpcCarrier` (in-process loopback, default features)** — `src/carrier.rs`. `serve` registers a `CarrierHandler` into an `RpcRegistry` (longest-prefix-match `NameTrie`); `invoke` builds a real Interest named `/svc/…/<op>` with the request as `ApplicationParameters` and dispatches it through the same registry. Optional security: `.with_signer(Arc<dyn Signer>)` signs requests; `.with_validator(Arc<Validator>)` fails closed on unsigned/invalid requests, sets `Invocation::requester` to the verified KeyLocator name, auto-enables a nonce `ReplayGuard(256, false)`, and verifies the response on the invoke side. Implements `Carrier` + `HintedCarrier` (forwarding hint), deliberately **not** `SelectCarrier`.

```rust
let carrier = RpcCarrier::new();
let svc = ServiceId::new("/svc/calc".parse().unwrap());
carrier.serve(&svc, Arc::new(CalcDispatch(Arc::new(CalcImpl)))).await.unwrap();
let client = CalcClient::new(carrier, svc);
assert_eq!(client.add(2, 3).await.unwrap(), 5);
```

**`FaceRpcCarrier` (real engine/network, feature `engine`)** — `src/face_carrier.rs`:

```rust
impl FaceRpcCarrier {
    pub fn client(consumer: Consumer) -> Self;    // ndn_app::Consumer
    pub fn server(producer: Producer) -> Self;    // ndn_app::Producer
    pub fn with_timeout(mut self, timeout: Duration) -> Self;   // default 4 s
    pub fn with_signer(mut self, signer: Arc<dyn Signer>) -> Self;
    pub fn with_validator(mut self, validator: Arc<Validator>) -> Self;
}
```

Full wiring (tests/face_carrier.rs, quoted in §1): consumer face + producer face on one engine, `engine.fib().add_nexthop(&svc_prefix, FaceId(2), 0)`, then

```rust
let producer = Producer::from_handle(p_handle, svc_prefix.clone());
let server = FaceRpcCarrier::server(producer);
server.serve(&svc, Arc::new(EchoDispatch(Arc::new(EchoImpl)))).await;

let consumer = Consumer::from_handle(c_handle);
let client = EchoClient::new(FaceRpcCarrier::client(consumer), ServiceId::new(svc_prefix));
```

For fleet use over UDP, this is exactly the §1 two-engine UDP setup with the producer behind engine A and the consumer on engine B (route `/svc/...` out the UDP face). Cargo: `ndn-rpc = { features = ["engine"] }`.

### Switching to the `ndn-ndnsf` carrier (four-phase, multi-provider)

`ndn-ndnsf` (`/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/ndn-ndnsf/`), carrier requires feature **`driver`**. Same service definition, zero changes — from `tests/macro_over_ndnsf.rs`:

```rust
use ndn_ndnsf::NdnsfCarrier;
use ndn_sync::{SvSyncConfig, SvsConfig, SvsPubSub};

// SvsPubSub::join(group, node_name, out_tx, in_rx, cfg) — the test fully meshes
// N nodes over tokio::mpsc channels (in-memory hub; see hub() in that test).
let bob = NdnsfCarrier::new(bob_ps, name("/muas/bob"), group.clone()).insecure();
bob.serve(&svc, Arc::new(GreeterDispatch(Arc::new(GreeterImpl { tag: "bob".into() })))).await.unwrap();

let alice = NdnsfCarrier::new(alice_ps, name("/muas/alice"), group.clone())
    .insecure()
    .token("utok");
let client = GreeterClient::new(alice, svc);
let one  = client.greet("ada".into()).await?;                       // FirstResponding
let many = client.greet_select("ada".into(), Strategy::All).await?; // Vec<(Name, String)> — all providers
```

`NdnsfCarrier` builder API (`src/carrier.rs`): `new(ps: SvsPubSub, node: Name, group: Name)`, `.ttl(secs)`, `.ack_window(Duration)`, `.token(String)`, `.signed(Arc<dyn Signer>, Arc<Validator>)`, `.insecure()`. **Secure by default = fail closed**: without `.signed(..)` or `.insecure()`, inbound four-phase messages are *rejected*. Op + request travel inside the four-phase payload envelope (not a name component); responses carry a status byte (OK/NOT_FOUND/ERROR) so errors are typed instead of timeouts. Implements `SelectCarrier`. Security caveat documented at `carrier.rs:49-63`: group membership *is* provider authorization today; `policy::ServicePolicy.providers` is not yet enforced on ACK acceptance.

More ndnsf tests worth copying: `tests/four_phase_over_svs.rs`, `tests/trust_validated_four_phase.rs`, `tests/service_node_multi.rs`, `examples/weather.rs` (`cargo run -p ndn-ndnsf --example weather --features driver`).

### W3C trace-context metadata slot?

**There is none.** `Invocation` has exactly `{ op, request, requester }` and `Response` exactly `{ producer, payload }` (`ndn-service-core/src/lib.rs:137-156`); `Carrier::invoke` takes only `(svc, op, request)`. Options that exist today, in order of cleanliness:
1. **In-band Frame field**: append a `traceparent: String` field to your request/response structs — the length-prefixed framing explicitly tolerates appended fields, so old peers ignore it.
2. Define a wrapper `Frame` (e.g. `Traced<T> { ctx: String, inner: T }`) and use it as every op's arg/return.
3. Not available: carrier-level headers, Interest ApplicationParameters side-channels (the whole request Bytes *is* the app-parameters), or Data MetaInfo access through this seam.

---

## 3. `ndf-apps` `AppRuntime`

Crate: `/Users/pmle/Documents/Dev/ndf-rs/refounding/ndf-apps/` (src: `runtime.rs`, `lib.rs`, `identity.rs`, `fabric.rs`, `follow.rs`, `store.rs`, `capture.rs`, `compute.rs`, `host_port.rs`, `render.rs`).
Test files (12, not 8): `serve.rs`, `deny_paths.rs`, `presence_liveness_bridge.rs`, `sdk_gaps.rs`, `gated_follow.rs`, `compute_socket.rs`, `bootstrap.rs`, `narration.rs`, `compute.rs`, `cluster_serving_policy.rs`, `restart_survival.rs`, `e2e_capture.rs`.

### Identity + address (`src/identity.rs`)

```rust
pub struct Identity { pub principal_ns: String, pub device: String, pub signing: ed25519_dalek::SigningKey }
impl Identity {
    pub fn new(principal_ns: &str, device: &str, signing: SigningKey) -> Self;
    pub fn svs_name(&self) -> String;              // "/alice/phone"
    pub fn public_key(&self) -> [u8; 32];
    pub fn chain(&self, app: &str) -> ChainAddress; // root = "<principal>/<app>/<device>"
}
pub struct ChainAddress { pub root: String, pub writer: String, pub writer_key: [u8; 32] }
// .root_name() -> Result<Name, BadName>; .root_bytes() -> Result<Vec<u8>, BadName>
```

### Attach + core surface (`src/runtime.rs`)

```rust
impl AppRuntime {
    pub fn attach(engine: ForwarderEngine, identity: Identity, cancel: CancellationToken) -> Self;
    pub fn attach_with_store(engine, identity, cancel, store: SharedStore) -> Self; // persistent seam

    pub async fn publish(&mut self, address: &ChainAddress, mime: &str, payload: &[u8])
        -> Result<PublishReceipt, PublishError>;
    // PublishReceipt { block: Hash, chain_seq: u64, svs_seq: u64 } — "Saved" = verified + stored + durable + announced.

    pub async fn serve(&mut self, address: &ChainAddress) -> Result<(), ServeError>;
    // boot-time resume+advertise of EXISTING history; ServeError::NothingToServe if store empty.
    pub fn serving(&self) -> Vec<String>;

    pub async fn follow(&mut self, address: ChainAddress, device: &DeviceDimensions, usage: UsageClass)
        -> Result<Follow, FollowError>;
    pub async fn follow_gated<C: ChainKind>(&mut self, address, chain_kind: C, device, usage)
        -> Result<Follow<C>, FollowError>;

    pub fn resolve(&self, address: &ChainAddress) -> Result<Vec<ResolvedBlock>, ResolveError>;         // cold: re-verify every Block
    pub fn resolve_trusted(&self, address: &ChainAddress) -> Result<Vec<ResolvedBlock>, ResolveError>; // hot: one Ed25519 verify per Block per process, signer-pin guarded
    pub fn head(&self, address: &ChainAddress) -> Result<Option<ChainNode>, ResolveError>;
    pub fn forks(&self, address: &ChainAddress) -> Result<Vec<ForkPair>, ResolveError>;

    pub fn identity(&self) -> &Identity;
    pub fn store(&self) -> &SharedStore;
    pub fn set_publisher_config(&mut self, config: PublisherConfig);  // ndn_app::PublisherConfig — persistent announce store (survive reboots)
    pub fn set_follow_config(&mut self, config: FollowConfig);
}
// ResolvedBlock { node: ChainNode, header: NdfHeader, payload: Option<Vec<u8>>, packet: Vec<u8> }
```

`publish` internally: builds `NdfHeader::sample_content_inline()` with `mime`, `seq = head+1`, `parent_ref = head.hash`; signs via `ndf_core::envelope::encode_signed_data(&root_uri, &header, Some(payload), &identity.signing)`; self-ingests through `verify_signed_data` + `ChainGate` (identical to wire path); flushes; announces over SVS via `ndn_app::Node::publish_with_config(root, svs_name, config)` → `Publisher::put(&packet) -> svs_seq`.

`Follow` (`src/follow.rs`): `attach()`, `fill_until(max_rounds: usize, done: impl FnMut(&SharedStore) -> bool) -> Result<Vec<ReplicationEvent>, FollowError>`, `decision() -> CarryDecision`, `presence()`. Presence decides Carry (attach now) vs Reach (attach on first fill).

### Minimal bootstrap the tests actually use (`tests/bootstrap.rs`)

```rust
let (engine, shutdown) = ndn_engine::EngineBuilder::new(EngineConfig::default()).build().await.expect("engine");
let cancel = CancellationToken::new();
let mut runtime = AppRuntime::attach(
    engine,
    Identity::new("/host", "solo", SigningKey::from_bytes(&[9u8; 32])),
    cancel.child_token(),
);
let addr = runtime.identity().chain("capture");          // or ndf_apps::capture::capture_address(..)
runtime.publish(&addr, "text/plain", b"hello").await.expect("Saved");
let notes = runtime.resolve(&addr).unwrap();
```

That is the whole identity/keychain bootstrap: **a raw `ed25519_dalek::SigningKey` — no keychain, no cert enrollment**. Trust = the follower pins `writer_key` inside the `ChainAddress` it follows. (Do NOT derive keys from names — explicit warning in `identity.rs:8-14`; the full genesis/TrustBundle/rotation exemplar is `cargo run -p ndf-policy --example trust_bootstrap`.)

### Two-node fabric wiring: `make_reachable` (`src/fabric.rs`)

```rust
pub fn make_reachable(engine: &ForwarderEngine, address: &ChainAddress, face: FaceId)
    -> Result<Reachability, BadName>;
// Installs: fib().add_nexthop(group=root, face, 10) + add_nexthop(data=writer, face, 10)
// AND the "multicast" strategy on both prefixes (so peer routes MERGE with local app-face
// registrations instead of shadowing them). Call on BOTH ends with each side's face toward the peer.
// Reachability { group: Name, data: Name }
```

Full two-region flow (bootstrap.rs, running inside ndn-sim; sim provides ONLY links):

```rust
fastrand::seed(67);
let kernel = VirtualKernel::new();
kernel.run(|k| async move {
    let mut sim = Simulation::new().kernel(k).seed(67);
    let a = sim.add_node(EngineConfig::default());
    let b = sim.add_node(EngineConfig::default());
    sim.link(a, b, LinkConfig::lan());
    let fabric = sim.start().await.unwrap();
    let cancel = CancellationToken::new();

    let mut rt_a = AppRuntime::attach(fabric.engine_of(a).unwrap(),
        Identity::new("/alice", "phone", SigningKey::from_bytes(&[5u8; 32])), cancel.child_token());
    let mut rt_b = AppRuntime::attach(fabric.engine_of(b).unwrap(),
        Identity::new("/bob", "laptop", SigningKey::from_bytes(&[6u8; 32])), cancel.child_token());

    let face_a_to_b = fabric.face_between(a, b).expect("link face a->b");
    let face_b_to_a = fabric.face_between(b, a).expect("link face b->a");
    for addr in [&alice_chain, &bob_chain] {
        make_reachable(&fabric.engine_of(a).unwrap(), addr, face_a_to_b).expect("wire a");
        make_reachable(&fabric.engine_of(b).unwrap(), addr, face_b_to_a).expect("wire b");
    }
    ndn_app::rt::sleep(Duration::from_millis(200)).await;

    rt_a.publish(&alice_chain, "text/plain", b"from alice").await.expect("Saved");
    let mut follow_b = rt_b.follow(alice_chain.clone(), &dims(), UsageClass::Active).await.expect("b follows alice");
    follow_b.fill_until(300, |s| s.chain_head(&alice_chain.root_bytes().unwrap()).map(|h| h.seq) == Some(0)).await.expect("converged");
    // rt_b.resolve(&alice_chain) now yields the Block.
    cancel.cancel();
    fabric.shutdown().await;
});
```

`DeviceDimensions`/`UsageClass` come from `ndf_policy::presence`; the tests use `DeviceDimensions { battery_pct: 90, metered_network: false, capacity_tight: false, attachment: AttachmentIntent::TravelsWith }` and `UsageClass::Active`.

Sidebar — `ndn_app::EngineAppExt` (`/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/app/ndn-app/src/engine_ext.rs`) is how AppRuntime touches the engine, and is useful directly: `engine.app_node(cancel) -> Node` (in-proc face + FIB registration; `node.serve(..)`, `node.fetch(..)`, `node.publish{,_with_config}(group, local_name) -> Publisher`, `publisher.put(payload) -> Result<u64 /*svs seq*/, AppError>`), `engine.register_producer(prefix, cancel) -> Producer`, `engine.app_consumer(cancel) -> Consumer`.

Also in ndf-apps: `serve_function`/`compute` (named pure functions, signed content-addressed results, CS memoization — see §1 UDP snippet's tail and `tests/compute*.rs`), and `HostPort::spawn` (a sync host thread driving an async region — `tests/bootstrap.rs` Gate 2).

---

## 4. `ndf-spark`

Crate: `/Users/pmle/Documents/Dev/ndf-rs/refounding/ndf-spark/` (src: `lib.rs`, `payload.rs`, `stream.rs`, `stream_core.rs`, `merkle.rs`, `rollback.rs`, `predicate.rs`, `tlv.rs`).
Tests: 8 integration test fns across 3 files (`spark_gates.rs` ×2, `stream_core_gates.rs` ×5, `real_socket_twin.rs` ×1) plus 15 unit tests in src — not 13.

Exports (`lib.rs`): `SparkPayload`, `RollbackHeader`, `StreamInstance` (= `[u8; 16]`), `SparkProducer`, `SparkSubscriber`, `SparkDeny`, and the transport-agnostic core: `SparkEmitter`, `SparkAcceptor`, `SparkCarrier`, `SparkSource`, `Emitted`, `AnchorVerdict`, `mint_instance`, `window_start`, `Hash = [u8; 32]`, `MIME_SPARK_CHECKPOINT`.

### The unit (`payload.rs`)

```rust
pub struct SparkPayload {
    pub instance: StreamInstance,   // per-producer-run id; regenerate each run (mint_instance())
    pub seq: u64,                   // ordering authority
    pub emitted_at: i64,            // Unix µs, advisory; caller stamps it (core owns no clock)
    pub data: Vec<u8>,              // app-schema'd
    pub rollback: Option<RollbackHeader>,  // { logical_tick: u64, author_id: Hash, tick_deadline_us: u32 }
}
impl SparkPayload {
    pub fn encode(&self) -> Vec<u8>;
    pub fn decode(buf: &[u8]) -> Result<Self, SparkDecodeError>;   // strict TLV, ascending tags
}
```

Deliberately NOT a Block: no envelope, no per-item signature (gate 1 in `tests/spark_gates.rs` asserts Spark ≤ half the Block's bytes and >10× faster than per-item signing).

### Transport seam — Spark is transport-injected

The core never names a transport (`stream_core.rs:15-17`). You bind it by implementing two one-method traits (`async fn` in trait — `#![allow(async_fn_in_trait)]`):

```rust
pub trait SparkCarrier { type Error; async fn carry(&mut self, bytes: &[u8]) -> Result<(), Self::Error>; }
pub trait SparkSource  { async fn next_spark(&mut self) -> Option<Vec<u8>>; }
```

Reference binding (`stream.rs`) rides **ndn-surface** (SHM gated streams), *not* the engine: `impl SparkCarrier for ndn_surface::NamedPublisher` (carry = `publish(bytes)`), `impl SparkSource for ndn_surface::NamedSubscriber` (`next_frame(|f| f.content.to_vec())`). A UDP socket, in-proc channel, or an engine face are equally valid bindings (`tests/stream_core_gates.rs` proves two bindings give byte-identical meaning; `tests/real_socket_twin.rs` runs the acceptor straight off `tokio::net::UdpSocket` datagrams).

### Producer / subscriber over a gated surface (`tests/spark_gates.rs`)

```rust
let name = Name::from("/ndf-spark-gate/123");
let secret = [9u8; 32];
let publisher = NamedPublisher::open_fanout_gated(name.clone(), &secret).await.unwrap();
let mut producer = SparkProducer::over(publisher, [1u8; 16]);   // (publisher, instance)

producer.emit(/*emitted_at*/ 1, &sample_bytes, /*rollback*/ None).await.unwrap(); // -> Ok(seq)
let signed_checkpoint: Option<Vec<u8>> = producer.checkpoint("/name/uri", &signing_key); // one signed Block per window
producer.close().await.unwrap();

let sub = NamedSubscriber::connect_gated(name.clone(), &secret).await.unwrap(); // wrong secret => Err (fd handshake gate)
let mut spark_sub = SparkSubscriber::over(sub)
    .with_predicate(expr, extract);           // optional in-set filter: FieldExtractor = fn(&SparkPayload) -> BTreeMap<String, PredicateValue>
let got: SparkPayload = spark_sub.next().await.unwrap().unwrap();  // Option<Result<SparkPayload, SparkDeny>>
spark_sub.is_complete();                       // Option<bool>: EOS vs abort
```

### Core (custom transports, e.g. an NDN face or UDP between drones)

```rust
let mut emitter = SparkEmitter::new(mint_instance(), Some(32 /*anchor_every*/));  // None = manual cut_checkpoint()
let out: Emitted = emitter.emit(now_us, &data, None);   // Emitted { seq, bytes, checkpoint: Option<Checkpoint> }
// send out.bytes over your carrier UNTOUCHED (acceptor hashes the bytes);
// when out.checkpoint is Some, sign it: checkpoint.to_signed_block(name_uri, &sk) -> Vec<u8> (a real Block)

let mut acc = SparkAcceptor::new();
if acc.accept(&datagram).is_some() {                    // judged SparkPayload
    if let Some(v) = acc.rejudge_pending() { /* AnchorVerdict resolved */ }
}
let cp = Checkpoint::from_block(&block_bytes, &pk)?;    // ndf_spark::merkle::Checkpoint
let verdict = acc.note_checkpoint(&cp);                 // Anchored{from_seq,to_seq} | Gapped{missing} | Tampered | StaleInstance
acc.has_pending();
```

Join-at-now contract: start at `window_start(live_seq, anchor_every)` or your first window is permanently `Gapped` (`stream_core.rs:48-57`). Checkpoint Blocks carry `MIME_SPARK_CHECKPOINT`; anchors typically ride a *different*, reliable channel (a chain via `AppRuntime::publish`) than the lossy sample lane — that split is exactly `tests/real_socket_twin.rs`.

---

## 5. `ndn-sim` (ndn-lab) scenarios

Crate: `/Users/pmle/Documents/Dev/ndn-workspace/ndn-sim/crates/ndn-sim/` (single crate in the workspace). `src/lib.rs` is the map; the builder lives in `src/topology.rs`.

### Building a Simulation with real ForwarderEngines + lossy links

```rust
use ndn_sim::prelude::*;   // or ndn_sim::{Simulation, LinkConfig, VirtualKernel, ...}

let mut sim = Simulation::new();                 // default WallClockKernel
let n1 = sim.add_node(EngineConfig::default());  // -> NodeId; a REAL ndn_engine::ForwarderEngine per node
let n2 = sim.add_node(EngineConfig::default());
sim.link(n1, n2, LinkConfig::lan());             // presets: lan() / wan() / lossy_wireless()
sim.add_route(n1, "/prefix", n2);                // optional static routes
let fabric = sim.start().await?;                 // -> RunningSimulation (alias: ndn_sim::Fabric)
fabric.shutdown().await;
```

`LinkConfig` fields (`src/sim_link.rs:21`): `delay: Duration`, `jitter: Duration`, `loss_rate: f64`, `bandwidth_bps: u64` — build your own for a lossy medium, or use `LinkConfig::lossy_wireless()`. Radio instead of point links: `sim.with_radio_medium(..)` / `sim.add_radio_node(..)` / `sim.broadcast_segment(&[ids], prefix)`; world/mobility via `sim.place_node(node, Position::xy(..))`, `sim.set_node_mobility(..)`, `sim.world(World)`.

Builder extras (`src/topology.rs:95-317`): `.kernel(Arc<dyn SimKernel>)`, `.seed(u64)`, `.add_app(node, AppSpec)`, `.add_strategy(node, prefix, "multicast")`, `.add_node_profile(NodeProfile)`, `.channel_buffer(size)`.

### Kernels + determinism

`DesKernel` (deterministic event queue), `VirtualKernel`, `WallClockKernel` (default), `RealTimeKernel` (hosts live devices / SITL). Pattern for virtual time (from ndf-apps `tests/bootstrap.rs` and ndn-sim tests):

```rust
fastrand::seed(67);                       // seed the global RNG too
let kernel = VirtualKernel::new();        // -> Arc<Self>
kernel.run(|k| async move {               // F: FnOnce(Arc<dyn SimKernel>) -> Fut; builds its own paused
    let mut sim = Simulation::new().kernel(k).seed(67);   // single-thread tokio rt; time auto-advances
    /* ... */
});                                        // panics after DEFAULT_RUN_CEILING of virtual time; run_capped() for Result
```

### App code on the nodes — how you get an engine handle

Two ways:
1. **`fabric.engine_of(node: NodeId) -> Option<ForwarderEngine>`** (`topology.rs:1060`) — hands you the node's real engine; attach anything (an `AppRuntime`, `engine.app_node(cancel)`, a `FaceRpcCarrier`, …). This is exactly what ndf-apps' bootstrap test does (§3). Companion introspection: `fabric.face_between(from, to) -> Option<FaceId>`, `fabric.route(node, &prefix, nexthop)`, `fabric.set_strategy(..)`, `fabric.explain_route(..)`, `fabric.face_stats(node)`, live topology ops `fabric.connect(a, b, LinkConfig)` / `spawn_node` etc. (via the `FabricControl` trait).
2. **Declarative `AppSpec`**: `sim.add_app(node, AppSpec)` before start, or `fabric.spawn_app(node, spec) -> Result<AppId>` live; poll `fabric.app_successes(app)` / `fabric.flow_stats(app)` (`src/app.rs` — producers/consumers with a `TrafficPattern`).

### `otel_export` setup (`src/otel_export.rs`, verified by `tests/otel_export.rs`)

```rust
use ndn_sim::{ControlPlane, MetricsLog, OtlpExporter, SimResponse, SimQuery};

let fabric = Arc::new(sim.start().await.unwrap());
let control = ControlPlane::new(Arc::clone(&fabric));
let SimResponse::Metrics(samples) = control.query(SimQuery::Metrics) else { panic!() };

let exporter = OtlpExporter::new("127.0.0.1:4318")       // OTLP/HTTP collector addr (impl Into<String>)
    .with_service_name("miniMUAS");                       // default service.name = "ndn-lab"
let status: u16 = exporter.export_metrics(&samples).await?;   // POST, OTLP/JSON body ("resourceMetrics",
                                                              //  gauges like ndn.cs.hit_rate / ndn.pit.depth)
```

Other exporter methods: `export_spans(&[Span])`, `export_captured_spans(..)` (pairs with `src/span_capture.rs`: `capture_engine_spans` / `SpanLog` capture the engine's own tracing spans on the virtual clock), `export_ip_metrics`, `export_fabric_gauges`, plus `*_payload()` string builders. `fabric.snapshot_metrics()` and `fabric.spawn_gauge_emitter(..)` also feed it. Control plane rides in-proc/NDN/TCP/WebSocket, and `SimMcp` exposes it as MCP tools.

### `mavlink` feature — SITL mobility adapter (`src/mavlink.rs`, `tests/mavlink.rs`)

Cargo: `ndn-sim = { features = ["mavlink"] }` (pulls `mavlink = "0.13"` with `["common", "udp", "std"]`; module and test are `#[cfg(feature = "mavlink")]`).

What it needs to run: a MAVLink telemetry endpoint (ArduPilot SITL / MAVProxy / real autopilot) streaming `GLOBAL_POSITION_INT` or `LOCAL_POSITION_NED`; a fabric on the **`RealTimeKernel`** (clock mode B — the autopilot is the clock master); node count + sysid base mapping.

```rust
use ndn_sim::mavlink::{MavlinkConfig, mavlink_link, mavlink_source};
use ndn_sim::{RealTimeKernel, Simulation, Position, VehicleCommand, ControlPlane, SimCommand};

let mut sim = Simulation::new().kernel(RealTimeKernel::new());
let node = sim.add_node(EngineConfig::default());
sim.place_node(node, Position::ORIGIN);
let fabric = Arc::new(sim.start().await.unwrap());

// positions IN only:
let (source, _reader) = mavlink_source(MavlinkConfig {
    endpoint: "udpin:127.0.0.1:14550".into(),  // listen for SITL telemetry
    reference: None,                            // None => first GLOBAL_POSITION_INT fix becomes the ENU origin
    base_sysid: 1,                              // sysid 1 -> NodeId(0); GCS (255) skipped
    node_count: N,                              // sysids beyond N ignored
}).unwrap();

// OR bidirectional (positions IN + actuator OUT on one connection):
let (source, _reader, actuator) = mavlink_link(cfg).unwrap();
let control = ControlPlane::new(Arc::clone(&fabric));
control.set_actuator(Arc::new(actuator));       // now SimCommand::Cosim { VehicleCommand::Goto{..} } flies the vehicle

let trace = fabric.drive_mobility(Box::new(source), Duration::from_millis(20), cancel).await;
// -> MobilityTrace; replay deterministically on DES via fabric.install_trace(&trace)
```

`VehicleCommand` variants handled by `MavlinkActuator`: `Arm/Disarm`, `Takeoff{alt_m}`, `Land`, `SetMode{mode}`, `ReturnToLaunch`, `Goto{x,y,z}` and `Velocity{..}` (ENU; converted to `SET_POSITION_TARGET_LOCAL_NED`). Pure helpers you can unit-test without a socket: `geo_to_enu`, `decode_position`, `default_node_of`. `tests/mavlink.rs` shows a complete fake SITL (mavlink crate + `udpout:`) if you need CI without ArduPilot. JSON codec for the control plane (works over an NDN Interest to `/localhop/sim/control`):
`{"command":{"cmd":"cosim","command":{"action":"goto","node":0,"x":10.0,"y":20.0,"z":0.0}}}`.

---

## 6. Identity/trust bootstrap — fastest working paths

- **ndf-apps publish/follow (in-proc or sim)**: zero ceremony. `Identity::new(ns, device, SigningKey::from_bytes(&[b; 32]))`; the follower's `ChainAddress` pins `writer_key: [u8; 32]`. No keychain, no certs, no engine `SecurityManager`. The engines in every ndf-apps/ndn-sim test run with `EngineConfig::default()` and **no** `.security(..)` — `SecurityProfile::Default` without a SecurityManager falls back to *AcceptSigned* (crypto-verify signatures, no namespace hierarchy) per `/Users/pmle/Documents/Dev/ndn-workspace/ndn-rs/crates/security/ndn-security/src/profile.rs:19-25`. Chain trust is enforced above the engine by `verify_signed_data` against the pinned key.
- **ndn-rpc secure carrier**: build an `Ed25519Signer::from_seed(&[9u8; 32], key_name)` and a `Validator::new(TrustSchema)` with an all-accepting schema rule + the signer's cert inserted into `validator.cert_cache()` — the exact "trust-anywhere" recipe is `open_schema()` / `validator_trusting()` in `/Users/pmle/Documents/Dev/ndn-workspace/ndn-ext/crates/service/ndn-rpc/tests/face_carrier.rs:103-125` (SchemaRule with `PatternComponent::MultiCapture("_")` for both data and key patterns; `Certificate { name, public_key, valid_from: 0, valid_until: u64::MAX, sig_type: SignatureEd25519, .. }`).
- **ndn-ndnsf**: `.insecure()` for tests/dev (mandatory call — default fails closed), `.signed(signer, validator)` for real; see `tests/trust_validated_four_phase.rs`.
- Real trust flow (genesis keys, TrustBundle, adoption, rotation): `cargo run -p ndf-policy --example trust_bootstrap` (`/Users/pmle/Documents/Dev/ndf-rs/refounding/ndf-policy/examples/`).

---

## 7. Gotchas

- **Async runtime**: everything is tokio underneath, but the engine abstracts it behind `ndn_runtime::Runtime` (`EngineBuilder::runtime(Arc<dyn Runtime>)`, default `ndn_runtime::default_runtime()`). `EngineBuilder::build()` is `async` and spawns tasks — call it from inside a runtime. Under `VirtualKernel::run` the kernel *owns* a paused single-threaded runtime; don't wrap it in `#[tokio::test]` (those tests are plain `#[test]`). Use `ndn_app::rt::sleep(..)` inside sim closures (bootstrap.rs does), so sleeps ride the sim clock abstraction.
- **Keep the `ShutdownHandle`**: dropping it is not shutdown; either hold it and `shutdown().await`, or `detach()` deliberately (compute_socket.rs pattern).
- **Feature flags**: `ndn-rpc` → `engine` for `FaceRpcCarrier`; `ndn-ndnsf` → `driver` for `NdnsfCarrier` (tests are `#![cfg(feature = "driver")]`); `ndn-sim` → `mavlink` for SITL, `bin` for the CLI; `ndn-engine` → `partitioned-fwd` for `DataPlane::Partitioned` (silently falls back to Shared otherwise); `ndf-apps` → `persistent-store` for `SharedBackend`/`BatchPolicy`, `trace-stderr` for the trace module; `ndn-service-core` → `default-features = false` for the `no_std` message layer, `seal` for ChaCha20-Poly1305 leaf sealing.
- **FIB shadowing is the #1 silent failure**: a route at a coarser prefix than the app-face registration lands in a different FIB entry, LPM picks one, and Interests go nowhere with zero errors. Use `ndf_apps::make_reachable` (derives group+data prefixes from the `ChainAddress` and installs multicast strategy on both) instead of hand-rolled `add_nexthop`+`add_strategy` (`fabric.rs:1-11` documents the bug class; `fabric.explain_route(..)` and the `fwd.fib` shadow diagnostic help debug).
- **Multicast strategy, not best-route, on shared prefixes**: best-route pins to the cheapest face and starves the rest when a publisher reappears on a new face (`fabric.rs:53-56`).
- **`ndn_service` macro constraints**: args must be plain identifiers; every arg/return type must impl `Frame`; impl blocks are plain `async fn` (macro rewrote the trait — adding `#[async_trait]` to your impl breaks it).
- **Secure `FaceRpcCarrier` drops unsigned requests silently** — the client sees a timeout, not `Unauthorized` (`face_carrier.rs` test `unsigned_request_dropped_by_secure_provider_over_engine`). Set `.with_timeout(..)` sensibly.
- **`RpcCarrier` is loopback-only** (`carrier.rs:9-16` scope note) — same registry on both sides. For anything cross-engine use `FaceRpcCarrier` or `NdnsfCarrier`.
- **Below-min-cost/goto-style silent drops exist elsewhere too**: unsolicited Data is `DropAll` by default — for broadcast/ad-hoc bearers set `unsolicited_data_policy(UnsolicitedDataPolicy::AdmitNetwork)` or overheard Data never caches.
- **Publisher seq spaces**: never share one persistent `DataStore` between two concurrently-live publishers with the same node name (`ndn-app/src/publisher.rs:41-43`); serve() is resume-not-replay — a history replay under seq-resume duplicates (`runtime.rs:309-314`).
- **UDP faces**: fixed peer per face (`recv` filters by peer addr); MTU default `DEFAULT_UDP_MTU`, recv buffer 9000 B; Linux-only `udp-recvmmsg` batch feature. compute_socket.rs sleeps 150 ms after `add_face` "faces settle (real clock)" before first fetch.
- **Determinism**: seed *both* `fastrand::seed(n)` and `Simulation::seed(n)`; `VirtualKernel::run` panics at the virtual-time ceiling (`DEFAULT_RUN_CEILING`) — use `run_capped` for a `Result`.
- **Nothing was newly compiled during this survey** — APIs were verified against source + existing tests; all repos have populated `target/` dirs from recent builds, and every snippet above is lifted from a checked-in test rather than invented.

---

## 8. Friction for maintainers

1. **No metadata/extension slot on `Invocation`/`Response`/`Carrier::invoke`.** Cross-cutting concerns (W3C trace context, deadlines, auth tokens beyond the ndnsf `user_token`) must be smuggled into each op's `Frame`. A `metadata: BTreeMap<String, Bytes>` (or one reserved appended frame field) on `Invocation`/`Response` would make tracing uniform across carriers.
2. **`ServiceId`/`OpId` inner fields are private with no `Display`** — fine, but the generated client stores `carrier` by value; there's no `&C` accessor, so one carrier can't be shared across two typed clients without wrapping it in `Arc` yourself (carriers aren't `Clone`).
3. **Feature-gate discoverability**: `NdnsfCarrier` simply doesn't exist without `--features driver`, and `FaceRpcCarrier` without `engine` — a bare `cargo add ndn-ndnsf` compiles but exposes almost nothing, with no compile error pointing at the feature. Doc-comment says it, rustc doesn't.
4. **Doc/reality drift in counts and names**: crate `ndn-sim` insists it's called "ndn-lab"; `ndf-apps` has 12 test files and `ndf-spark` 8 integration tests (the task brief said 8 and 13 — whatever those numbers once were, they've drifted). `RpcCarrier`'s header says "The `#[ndn_service]` macro (planned)" in `ndn-service-core/src/lib.rs:12` while the macro has shipped and has tests.
5. **Two "publish" verbs, three layers**: `ndn_app::Node::publish` (SVS publisher), `AppRuntime::publish` (chain Block), `ndn_surface::NamedPublisher::publish` (stream frame), plus `ndn-service-core::publish` (embedded leaf). All correct locally; navigating them cold is rough.
6. **Engine `add_face` takes a raw `CancellationToken`** with no guidance that it should be a child of the shutdown token; tests pass `CancellationToken::new()`, which means faces outlive `ShutdownHandle::shutdown()` semantics unless you know to wire it.
7. **`InProcFace::new(FaceId, ..)` invites collisions** — you must hand-pick unique ids or remember `builder.alloc_face_id()` / `engine.faces().alloc_id()`; nothing enforces uniqueness at insert time.
8. **`NdnsfCarrier` provider authorization gap** is documented only in a doc comment (`carrier.rs:49-63`): any trusted group member may serve any service; `ServicePolicy.providers` exists but is unenforced on ACK acceptance. For a fleet: one SVS group per trust domain, or fix the ACK path.
9. **Silent-failure modes cluster around the FIB** (shadowing, best-route starvation, 3 m-AGL-style gates elsewhere in this ecosystem): `make_reachable` fixes the app layer, but a raw engine consumer gets no warning. `explain_route`/`fib shadow diagnostic` exist but you have to know to ask.
10. **Spark checkpoint retention (FS-7b) is explicitly unowned** (`stream_core.rs:26`) — anchors accumulate wherever you publish them; plan your own retention.
11. **`ndn-service-core` framing is length-prefixed-in-order, not tag-per-field TLV**, despite the trait docs saying "MUST use TLV with skippable unknown fields"; the codec-revision caveat is buried at `lib.rs:295-301`. Appending is safe; anything else is not — easy to get wrong when evolving fleet message schemas.
12. **MavlinkReader thread is never joined by design** (blocking `recv()`); on teardown it lingers until the next frame or process exit — harmless in tests, worth knowing in a long-lived agent process.
