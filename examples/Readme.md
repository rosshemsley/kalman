# Examples

The examples are kept separate from the main module to allow them to use different dependencies from the main package.

You can compile and run the examples using

```bash
go get github.com/rosshemsley/kalman/examples/...
```

This will install the example binaries for running: `trajectory-example` and `time-series-example`.
Running each of these produces a `plot.png` file with the output.

# Time Series Example
In the time series example, we model an unknown 1D time series using a Browniain model.
The Filtered result is drawn.

![Alt text](images/time_series_example_plot.png?raw=true "time series example")

# Trajectories Example
This example smoothes a trajectory with a constant velocity model.
Given some noisy 2D points sampled at non-uniform times, we smooth the trajectory
with both a Kalman Filter and Kalman Smoother, and then draw the results.

![Alt text](images/trajectory_example_plot.png?raw=true "trajectory example")
