# Overview
This repository contains all the code needed to complete the Model Predictive Control

[//]: # (Image References)

[drive1]: images/drive1.png "drive1"
[drive2]: images/drive2.png "drive2"
[model]: images/model.png "model"

## Build

```
./go.sh
```

## Run

```
./run.sh
```

Here are some screenshot from simulator:

![drive1][drive1]

![drive2][drive2]

Here's the [link to my video result in Youtube](https://youtu.be/RUyy2Wzb5fk)


## Reflection

### The Model

This section describes the model in detail. This includes the state, actuators and update equations.

We are using a `Kinematic` model. It is a comparable simple model. Here is a screenshot for the formula of this model

![model][model]

The Vehicle State and Actuator outputs:

* Vehicle's x and y coordinates, `x` and `y`
* Vehicle's orientation angle, `psi`
* Cross Track Error, `cte`
* Error in PSI, `epsi`
* Steering Angle, `delta`, the output from Actuator
* Acceleration, `a`, also the output from Actuator

Here is the code for update function:

``` cpp
fg[0] = 0;

for (int i = 0; i < N; i++){

  fg[0] += g_cte_factor * CppAD::pow(vars[g_cte_start + i] - g_ref_cte, 2);
  fg[0] += g_epsi_factor * CppAD::pow(vars[g_epsi_start + i] - g_ref_epsi, 2);
  fg[0] += CppAD::pow(vars[g_v_start + i] - g_ref_v, 2);

}

// Update Change Rate
for (int i = 0; i < N -1; i++){

  fg[0] += g_delta_factor * CppAD::pow(vars[g_delta_start + i], 2);
  fg[0] += g_a_factor * CppAD::pow(vars[g_a_start + i], 2);

}

// Update the Sequential Actuations
for (int i = 0; i < N -2; i++){

  fg[0] += g_diff_delta_factor * CppAD::pow(vars[g_delta_start + i + 1] - vars[g_delta_start + i], 2);
  fg[0] += g_diff_a_factor * CppAD::pow(vars[g_a_start + i + 1] - vars[g_a_start + i], 2);

}
```


### Timestep Length and Elapsed Duration (N & dt)

This section discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. 

Additionally the details the previous values tried.

Here is the value for `Timestep length N` and `dt`:

``` cpp
size_t N = 10;
double dt = 0.1;
```

`N` is the number of timesteps in the horizon. And if we increase `N`, it will increase more iteration (computation) for the model.
I tried N from 10 to 30, which larger N will produce an erratic result.

`dt` is the time gap between the acuations. If we choose a large `dt`, it will descrese the accuracy of the model. 
I also tried `dt` from 0.001, 0.2, etc, even a small change of `dt` will lead to an erratic result.

So I fianlly choose the value suggested in the quiz section.



### Polynomial Fitting and MPC Preprocessing

I transformed the coordination system of waypoints as preprocess, which from map coordination system to the vehicle coordination system. The benefit of this transformation is to simplify the polynomial fitting process, which origin from (0,0) and orientation angle from 0.



### Model Predictive Control with Latency

The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Here is the code for latency processing after the transformation with MPC model.

``` cpp
double t_latency = 2.5;
double t_number_latency = 20;

for (int i = 1; i < t_number_latency; i++){

  next_x_vals.push_back(t_latency * i);
  next_y_vals.push_back(polyeval(t_paras, t_latency * i));

}
```




