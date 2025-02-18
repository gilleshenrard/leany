# Leany Project PCB Documentation

**Revision:** 0.2

Welcome to the Leany project's PCB documentation. This repository contains all the necessary files and information related to the PCB schematics and layouts for the Leany project.

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Directory Structure](#directory-structure)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This documentation provides an overview of the PCB schematics and layouts used in the Leany project. It includes detailed information on the design, components, and assembly instructions.

## Generate the production files

1. Download KiCad from the [official website](https://kicad.org/).
2. Go to the pcb/ directory
3. Run the generation script located under resources/

## PCB fabrication

- Once the production files are generated, head to [JLCPCB](https://cart.jlcpcb.com/quote?orderType=1&stencilLayer=2&stencilWidth=100&stencilLength=100)

- Upload the zipped Gerber file, then, when the website is done processing it, choose the following non-default options :
  - Surface finish : LeadFree HASL
  - Enable PCB Assembly
- Then, click Next until arriving at the Bill Of Materials page.
- Upload the BOM and CPL files and click "Process BOM&CPL".
- Check that no error is shown on the components list page, and click Next.
- On the Components Placement page, move the components which are misplaced, then click Next.
- On the final page, click Save to Cart.

## Directory Structure
```
pcb/
├── 3d-models
└── resources
    └── generate_production_files.sh
```

### `3d-models`
This directory contains the additional 3D models needed to export the PCB as an STL file

### `resources`
Contains additional resources :
- `generate_production_files.sh` : A script to automatically generate and organise production files, with respect to JLCPCB standards.

## CI/CD workflows
The PCB files are instected by a [Github Action](https://github.com/gilleshenrard/leany/actions/workflows/pcb_production_files.yml) upon push and pull request to run the **Electrical Rules Check** and **Design Rules Check** ensure the production files are easily generated.

## Contributing

Contributions to improve the PCB design and documentation are welcome. If you have any suggestions or improvements, please create a pull request or open an issue on the repository.

## License

This project is licensed under the MIT License. See the [LICENSE](https://github.com/gilleshenrard/leany/blob/main/LICENSE) file for more details.
