# ⚡️Kalman Filter

[![](https://godoc.org/github.com/rosshemsley/kalman?status.svg)](https://godoc.org/github.com/rosshemsley/kalman)

A package implementing Kalman filtering and smoothing for continuous time-indexed models with non-uniform time transitions.
A collection of models are also provided, including a constant velocity motion model and a Brownian motion model.
Support for prediction, filtering, smoothing and sensor fusion are currently implemented.

## Design
The package has two main components

### Kalman Filter/Smoother
These implement state space estimation for a given model and measurements.


### Model

The model (such as `models.LinearModel`) provide common example models for modelling
time series data. For example `models.ConstantVelocityModel`, models the position
and velocity of a particle over time.


## Examples

![Alt text](examples/images/time_series_example_plot.png?raw=true "time series example")

For runnable examples, see `/examples`. Below, a full runnable example of filtering
a noisy time series. The model here is just a Brownian motion model, which assumes
that the time series represents a hidden value that is static apart from a Brownian noise
component.

```go
package main

import (
	"fmt"
	"time"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
)

func main() {
	var t time.Time
	values := []float64{1.3, 10.2, 5.0, 3.4}

	model := models.NewSimpleModel(t, values[0], models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     1.0,
		ObservationVariance: 2.0,
	})
	filter := kalman.NewKalmanFilter(model)

	for _, v := range values {
		t = t.Add(time.Second)
		filter.Update(t, model.NewMeasurement(v))
		fmt.Printf("filtered value: %f\n", model.Value(filter.State()))
	}
}
```
