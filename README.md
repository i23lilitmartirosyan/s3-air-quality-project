# Air Quality Monitoring System — S3 Project

This repository contains all the code and files for our S3 Air Quality Monitoring Project.  
It includes:

- Arduino code for ESP32 and sensors (BME280, SCD30, SGP40, SPS30)
- AWS Lambda code for receiving and storing JSON data
- PCB design files (EasyEDA exports)
- JSON data samples from S3
- Documentation and project structure

My main contribution in this project:
- SPS30 PM sensor implementation
- Parsing sensor frames and CRC validation
- Integrating PM data into JSON payload sent to AWS
