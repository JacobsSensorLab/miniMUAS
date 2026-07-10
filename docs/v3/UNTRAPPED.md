# Liberated, but still thinking like a prisoner

*A working statement from the miniMUAS project — for humans and machines
building on NDF. Written 2026-07-10, during the third feedback round of
the v3 rebuild. Keep, share, propagate, revise.*

## The observation

We moved the data out. Telemetry, journals, missions, calibrations,
strategies — named, signed, content-addressed, replicated by policy,
verifiable by anyone the trust frontier admits. The silo walls are down.

And yet the data still *clumps in the shapes of its old cells*. A
telemetry sample dresses for the dashboard that first displayed it. A
recording duplicates a journal because a replay slider once needed a
file. A wire schema outlives its transport and becomes the product. Each
app authors its own private vocabulary for things the whole fleet — the
whole ecosystem — means identically. The data has been liberated, but it
has no connections, doesn't know it can move, and keeps returning to its
old boundaries like it never left.

This is not a tooling failure. The substrate is ready. It is a *habit*
failure — ours, and, we suspect, every early NDF user's. The comfortable
path (a web stack, a bespoke schema, an app that owns its data) is always
one sprint away, and every time we take it we pour fresh concrete in the
shape of the walls we tore down.

## Evidence from our own build

We are not pointing fingers outward. In one day of honest review we
found in our own v3:

- A dashboard that *consumes* NDF but is *made of* WebSocket messages —
  the schema is a shadow-silo; unplug the socket and the surface knows
  nothing.
- Recordings kept beside journals: two truths, one of them redundant the
  moment replay learned to fold a chain.
- Kinds authored per-app that should have been shared strata; the
  operator suite next door independently authored five more, then had to
  re-ring them into a neutral core when a second consumer (us) appeared.
- A console whose docking path hardcoded its own five types — adoption
  was *supposed* to be the matcher's job, and the silo reflex snuck back
  in through a `match` statement. (Fixed same-day once named. That is the
  encouraging part.)

The counterexample that shows the ceiling: the artifact generator. One
mission dataset resolved by name; a report, a slide deck, a live demo,
and a cross-run comparison — four audiences, zero copies, provenance by
association with hashes underneath. Nothing about it was harder than the
silo version. It was only *unfamiliar*.

## The direction: malleable, not modular

Ink & Switch's essay on malleable software names the destination: tools
people reshape while using them, instead of applications that own their
users' contexts (https://www.inkandswitch.com/essay/malleable-software/).
NDF is the material science for that vision: the semantic manifest says
what data *means*; the render contract says what a lens *can honestly
express*; the verdicts (Express / Approximate / Refuse / Unresolved) make
degradation a designed experience instead of a broken widget. A user who
composes a gauge from a drone's namespace is not configuring our app —
they are building *their* surface out of *meaning*, and a stranger can
interrogate every element of it because the meaning travels with the
data.

What remains is not invention. It is the incremental realization —
surface by surface, kind by kind — and the discipline not to fall back.

## The cause beneath the effect

*(Added round-3½, after the fair challenge: the clumping is a symptom —
name the disease or the recognition is for naught.)*

Why does liberated data keep clumping in cell shapes? Not one cause but a
gradient, and every slope of it points the same way:

1. **The unit of making is still the app.** Repos, crates, frameworks,
   deadlines, and credit all bound at the application. Data design
   inherits those boundaries — Conway's law operating on *meaning*. The
   silo is not chosen; it is the shadow cast by the unit of work.
2. **Minting is cheaper than reusing.** One derive macro mints a private
   kind in ten seconds. Reusing a shared vocabulary costs discovery,
   reading, dependency wiring, and sometimes a conversation with another
   human. The macro is cheaper than the conversation, every time, so the
   ecosystem mints. (Our own duplicate coord types; the suite next door
   authoring five parallel kinds — nobody was lazy, everyone followed
   the gradient.)
3. **The commons is invisible at the moment of authoring.** No tool shows
   you, as you type `struct TelemetrySample2`, that a matching stratum
   already exists with nine of your twelve fields. The promotion litmus
   (D-36) lives in the spec; nothing affords it in the editor. You cannot
   reuse what you are never shown.
4. **Meaning is exhaust, not deliverable.** Kinds are byproducts of
   feature work, shaped by one feature's needs on one feature's deadline.
   No task says "author the vocabulary"; no review ritual owns it; no
   credit attaches to it.
5. **The trained prior.** Every developer — and every machine assistant —
   learned software from a corpus where frontend/backend/schema-per-app
   IS the shape of software. Under time pressure we all sample from the
   prior. The comfortable path is comfortable because it is the learned
   distribution, and it re-trains itself with every repo it generates.

Beneath all five: **the silo is locally optimal.** It ships today with no
coordination tax. The commons is globally optimal but front-loads its
cost. Exhortation cannot beat a gradient — only restructuring can:

- **Make reuse cheaper than minting.** Stratum search and one-command
  adoption at authoring time; a lint that says "this new kind matches
  `fleet-semantics/telemetry-sample` on 9/12 fields — adopt or justify."
  D-36 as tooling, not doctrine.
- **Make meaning a deliverable.** Vocabulary authoring is a named task
  with review (the atelier/countersign ritual moved into the everyday
  loop) and visible credit.
- **Make discovery ambient.** Surface catalogs, namespace browsers, and
  "N other consumers read this kind" counts in the places builders
  actually look.
- **Measure the clump.** Kinds-per-consumer ratios, single-reader chains,
  dead vocabularies — a silo-smell report fed from the chains themselves,
  because the substrate can audit its own liberation.
- **Retrain the prior on purpose.** Ship meaning-first starters and
  worked examples (the artifact generator, not the web-stack template) so
  the sampled default changes — for humans and for the machines that
  assist them.

The disciplines below manage the symptom. These five moves attack the
cause. Both are needed; only the second flips the gradient.

## Disciplines (the checklist we now hold ourselves to)

1. **Name data for what it means, not where it appeared.** If the kind
   name mentions a surface, a screen, or an app, it is wearing its cell
   uniform.
2. **Author the vocabulary before the surface.** A widget that precedes
   its stratum will define the stratum in its own image.
3. **Every surface is a lens; no surface is a home.** If deleting a
   frontend would orphan data, the data was never free.
4. **One truth per fact.** Derived artifacts (recordings, caches,
   spreadsheets) must be re-derivable from chains and must say so.
5. **Degrade by contract, never by breakage.** Express → Approximate →
   text-and-value baseline. Absence is information.
6. **Associations for humans, hashes underneath.** Lead with "which
   settings produced this"; keep verification one disclosure level down.
7. **Config, strategy, layout: records.** If it changes behavior and
   isn't a signed record on a chain, it is invisible history.
8. **Browsing is a right.** Any authorized surface can discover any
   namespace's kinds, capabilities, and contracts — device-agnostic,
   subject-patterned, never hardcoded to the drone it was built beside.
9. **Watch for the silo smells**: a schema only one process understands;
   a "temporary" file becoming load-bearing; an enum a foreign kind
   cannot enter; help text that lives in the app instead of the manifest.
10. **When you catch the reflex, name it in public.** Same-day fixes
    happen when the finding is specific and shared; concrete cures only
    while wet.

## To the other early users

If your data still clumps — if your beautiful chains feed exactly one
screen each — you are where we were this morning. The substrate is not
waiting on more features. It is waiting on us to stop building cells out
of habit. Compare notes, trade strata, adopt each other's kinds, and file
the specific friction the day you feel it. The feedback loops on this
stack are running same-day right now. That is rare, and it is the whole
game.

*— miniMUAS v3, third feedback round. This document is itself named data;
revise it, fork it, and cite what you change.*
