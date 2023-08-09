# Cache System Design - Lab-6

Welcome to the Cache System Design for Lab-6 of the UCR IE-0424 course! This repository provides Verilog code for implementing a cache system that interfaces with a RISC-V CPU (picorv32), a memory unit (mem_prin), and a seven-segment display (seven_segment_hex) on a Nexys 4 DDR (or Nexys A7) FPGA board.

## System Overview

The cache system is designed to work with the following components:

- **picorv32**: A RISC-V CPU core.
- **mem_prin**: A memory unit responsible for memory read and write operations.
- **seven_segment_hex**: A seven-segment display interface.
- **clock_divider**: A clock divider module for generating a slower clock.

## Cache Direct-Mapped Implementation

The `cache_directo` module represents a direct-mapped cache with the following features:

- Tag, index, and offset calculation for address decoding.
- Handling of cache read and write operations (hit and miss cases).
- Dirty bit management for write-back operations.

## Memory Principal (mem_prin)

The `mem_prin` module simulates memory access and control, providing the following functionalities:

- Simulation of memory read and write operations.
- Control over memory access timings and responses.

## Seven-Segment Display (seven_segment_hex)

The `seven_segment_hex` module interfaces with a seven-segment display to show hexadecimal output. It is used to display data from the cache.

## Clock Divider (clock_divider)

The `clock_divider` module generates a slower clock signal from the main clock, which is used for driving the seven-segment display. This helps to slow down display updates for better visibility.
