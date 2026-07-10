# miniMUAS → waterline — feedback #2: the promised multi-instrument test case, delivered

Same-day follow-through on both sides: you fixed our four blockers at
`7a45e03`; we built the docking adapter against it. The live
multi-instrument test case we offered now exists as code you can run:
`uas-fleet-instrument` (JacobsSensorLab/uas-fleet, `2006064`),
acceptance suite included.

## Your C5 fix, confirmed from the consumer side

Three `FleetInstrument`s (wuas-01 / iuas-02 / ruas-01) over one engine
into one `ConsoleCore` at your HEAD: 32 panels docked; each battery panel
holds only its vehicle's samples (histories 3/1/1, zero interleaving);
tracks route by subject exactly; journal heads advance per-root only; a
deliberately broad `/muas` series panel received **zero** facts
(most-specific-wins holds); rtl preflight+receipt fold onto the issuing
vehicle's verb panel only, and the receipt cites the honored preflight
hash; zero forks across 15 chains. The routing semantics are right.
Ask-4's `add_document`/`admit` path also works: our pinned lifecycle
stratum (`6770f6dd…`) installs and its airframe cards dock.

## Five new items (each proven in our code/tests)

1. **FloorTable matches verb names exactly; docs say prefix.** A fleet
   floor therefore needs one block per vehicle (our shipped
   `floor/uas-fleet.floor` duplicates rulings per vehicle). Either fix
   the docs or (better for fleets) support subject-prefix rulings.
2. **No "interim-gated" safing class.** Our `takeoff` borrows the
   `terminal` arm solely to get the MissingInterim gate — a vocabulary
   stretch that will read as "takeoff is terminal" to anyone auditing the
   floor. An explicit `interim-required` class would say what's meant.
3. **Floor doesn't dispatch on `verb-emergency-stop`** (renders P8
   amber), so our `terminate` ships without the e-stop claim. When
   EnforcementKind::EmergencyStop gains dispatch, we'll claim it.
4. **`FoldPoint::At` admits nothing on command chains** — preflight/
   receipt records lead with the verb hash, not an instant, so time-
   scrubbing a command chain yields an empty fold (assertion-proven).
   Either stamp commands with `measured:instant` first (WL-7 amendment)
   or teach the fold to find the instant field positionally per-kind.
5. **sextant-tty has no `--stratum`/`--contract` flags** — consumer
   vocabularies (our lifecycle stratum) currently require an embedding
   console rather than the stock TTY. CLI injection finished onboarding
   for chains (Ask 3); the same is needed for strata.

## Operational detail worth keeping

Your `--follow` grammar costs five follows per vehicle (instrument,
battery, track, journal, receipts). Fine at 3 vehicles; at 10+ a
`--follow-instrument <principal>,<writer>,<key>` macro expanding to the
declared streams would keep command lines humane.

Full wiring recipe (exact sextant-tty command line for a miniMUAS
deployment) lives in our crate docs and the floor-file header. Standing
offer continues: point your next round at `uas-fleet-instrument`'s
acceptance suite — it is now the fastest way to regression-test fleet
routing against a real external publisher.
