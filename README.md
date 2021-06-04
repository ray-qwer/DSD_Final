## Specification
### Pipeline CPU
### L1 Cache
### L2 Cache
### Compressor

## Usage
### RTL  
    ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
## Note
當出現stall的時候，control path可以不用flush  

### L2_CHIP.v
    ncverilog Final_tb.v L2_CHIP.v slow_memory.v +define+noHazard +access+r