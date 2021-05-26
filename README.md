### Frame format

Example:
`LD+${byteMSB}${byteLSB}#${crc}`

where:

- ${byteMSB} - left 8 bits
- ${byteLSB} - right 8 bits
- ${crc} - CRC

### avr-gcc options

- `-Os`

  The `-Os` option will tell the compiler to optimize the code for efficient
  space usage (at the possible expense of code execution speed)

source: http://www.elec.canterbury.ac.nz/intranet/dsl/p30-avr/doc-include/avr-libc/avr-libc-user-manual-1.6.1.pdf
