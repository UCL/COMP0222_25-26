# COMP0222 Public

This repo contains the code for the lab exercises and courseworks for COMP0222, taught in the spring 2026. Common code is drawn from a fork from the ```COMP0222/0249``` repo.

The code has been tested on MATLAB R2024b onwards on Windows, Linux and Mac (both Intel and M1) platforms.

To install the code, you should clone this repo:
```
git clone https://github.com/UCL/COMP0222_25-26.git
```

Once you've downloaded the software, open MATLAB and change to the `COMP0222_25-26` directory. Run the command:

```
setup
```
And you should be good to go.

The code uses one native library (``suitesparse``) for fast inversion of sparse matrices. It is only used in the factor graph to perform marginalization (covariance extraction). A copy of this library is in the `Libraries/ThirdParty/sparseinv` directory. Pre-compiled mex files for Windows, Mac and Linux are provided. If these do not work (e.g., difference in MATLAB versions), make sure your system is set up to compile mex files, delete the pre-compiled mex files  (`sparseinv_mex.mex*`), and run `install_sparseinv`. This should automatically build a new mex file. If the build process fails, please contact the module leads and TAs.
