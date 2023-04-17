# Walter Arduino software

## Introduction 

This repository contains the libraries to use Walter in the Arduino ecosystem.
All libraries and software are written with the same design strategies:
 - Keep it simple
 - Be as efficient as possible
 - Keep the code documented
 - Do not change underlying frameworks

This means that Walter is very easy to use in the Arduino ecosystem as you can
use the `ESP32S3 Dev Module` board from the standard 
[Arduino core for the ESP32-S3](https://github.com/espressif/arduino-esp32).
More information about how to set this up can be found in the 
[documentation repository](https://github.com/QuickSpot/walter-documentation).

## Contributions

We welcome all contributions to the software via github pull requests. Please
take the design strategies in mind when contributing. 

## License

All software is available under the 'DPTechnics 5 clause' license. This is 
essentially the same as the `BSD-3-Clause` license with the addition that
binaries of which the source code is not open should run on a Walter board from
DPTechnics bv.