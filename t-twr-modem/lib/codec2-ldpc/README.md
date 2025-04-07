# README.md

LDPC-related codes, selectively copied from https://github.com/drowe67/codec2

- H_1024_2048_4f: rate 1/2 code for the full 1024 data bits
- HRAa_1536_512: used as a rate 2/3 code by shortening to 1024 data bits

The ```rnd_1024_256``` code was created by generating a few hundred
LDPC codes for 1024 data and 256 partity bits, and testing each code
for its strength on random data. The best code was retained. It may be
worth hunting for even better rate 4/5 codes.

The decoding routing (in mpdecode_core.c) was split into a setup phase
(wiring the static memory layout of the MP graph), and the actual
initialization with the LLR values before iterating.

April 2025, HB9HUH/K6CFT
