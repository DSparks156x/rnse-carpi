# References

- `PQ35_46_ICAN.dbc`: infotainment CAN, the bus carpid sits on.
- `PQ35_46_ACAN.dbc`: drivetrain CAN.

Both cover the whole PQ35/46 family: VW, Skoda and Audi, with every nav unit,
cluster and option. An Audi with an RNS-E sends far fewer messages. Before
wiring a signal, check that its frame appears in a capture from the car, not
just in the DBC. Decoders are written by hand in `crates/carpi-signals`, using
these files as a reference (see `docs/PLAN.md`).
