
## Day 7 — C++ classes
Built a tiny `BrakingDecision` class alongside free functions in `day5_braking/`, plus a few asserts.
New to me: public/private, constructor initializer list, const methods, header vs source separation.
Still practicing: writing a ctor + arg parsing without peeking.

## Day 8 — CSV I/O
Added `--csv` mode to the braking CLI; learned file I/O (`ifstream`), line/token parsing (`getline`, `stringstream`), and safe number conversion (`stod`). Added tiny fixtures + a shell test that diffs expected vs actual output.

## Day 10 — Packaging & polish
Added MIT license, install rules (CMake), local install layout, and tagged v0.1.0. All CSV tests green.
