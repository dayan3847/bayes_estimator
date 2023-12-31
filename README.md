# Bayes Estimator

<img src="https://uady.mx/assets/img/logo_uady.svg" width="15%" alt="uady">

### <font color='orange'>Universidad Autónoma de Yucatán</font> _Facultad de Matemáticas_

**Teacher:** Dr. Arturo Espinosa Romero <[eromero@correo.uady.mx](mailto:eromero@correo.uady.mx)>

**Student:** Ing. Dayan Bravo Fraga <[dayan3847@gmail.com](mailto:dayan3847@gmail.com)>

# Practice 1: Proof recursive _μ_ and _σ_.

Empirical demonstration of the equivalence of the standard and recursive functions of _μ_ and _σ_.

## GitHub: [Practice 1](https://github.com/dayan3847/bayes_estimator/tree/main/practice1-mu_experiment) [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/dayan3847/bayes_estimator/blob/master/practice1-mu_experiment/practice1.ipynb)

## Steps:

* Generate a random _μ_ and _σ_.
* Generate a quantity _N_ of random samples from a normal distribution with the parameters _μ_ and _σ_.
* Calculate the mean and variance of the samples generated in a non-recursive way.
* Calculate the mean and variance of the samples generated in a recursive way.
* Graph the mean and variance of the samples generated in a non-recursive and recursive way.
* Compare the results obtained.

## Results

<img src="./practice1-mu_experiment/img/r.png" alt="h" width="1489">

# Practice 2: Analytical demonstration of _μ_ iterative and recursive.

Analytical demonstration of the equivalence of the iterative and recursive functions of _μ_.

## PDF Doc: [Practice 2](https://raw.githubusercontent.com/dayan3847/bayes_estimator/main/practice2-mu_demonstration/practice2.pdf)

## Preview:

<img src="./practice2-mu_demonstration/src/img/preview20.png" alt="h" width="1279">

## Conclusion:

It has been demonstrated using the method of mathematical induction.
We have shown that if the formulas are equivalent for _n = k_, then they are also equivalent for _n = k + 1_.
Since we have already shown that they are equivalent for _n = 1_ (base step),
we can conclude that the iterative and recursive formulas for calculating $\mu$ are equivalent for any value of n,
therefore, they are equally valid.

# Practice 3: Presentation of concepts

## Concepts:

* Expected Value
* Conditional Probability

## [PDF Presentation](https://raw.githubusercontent.com/dayan3847/bayes_estimator/main/practice3-ppt_concepts/practice3.pdf)

# Practice 4: Mouse Tracking with Kalman Filter

Manual implementation of a Kalman filter for Mouse Tracking *without using the OpenCV KalmanFilter class*.

### [Mouse Tracking with Kalman filter with Position and Velocity](./practice4-kf_mouse/KalmanFilterMouseXV.py)

<img src="./practice4-kf_mouse/example/exPV.png" alt="h" width="400">

### [Mouse Tracking with Kalman filter with Position, Velocity and Acceleration](./practice4-kf_mouse/KalmanFilterMouseXVA.py)

<img src="./practice4-kf_mouse/example/exPVA.png" alt="h" width="400">

# Practice 5: Ball Tracking with Kalman Filter

*The documentation and code is found in the following repository*

### https://github.com/dayan3847/KalmanFilterBallTracking
