# ⚡️Kalman Filter

[![](https://godoc.org/github.com/rosshemsley/kalman?status.svg)](https://godoc.org/github.com/rosshemsley/kalman)

A package implementing the Kalman Filter equations for continuous time-indexed models (i.e. a time series),
along with a collection of models for use in common cases, such as constant velocity motion and Brownian motion.
Support for prediction, filtering, and sensor fusion are currently provided.



## Design
The package has two main concepts,

### Filter
This manages the filtering and prediction of a hidden state using the KalmanFilter.

### Model

The model (such as `models.LinearModel`) provide common example models for modelling
time series data, such as `models.ConstantVelocityModel`, which models the position
and velocity of a particle in the hidden state of the filter.

## Examples

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

	for _, v := range values[1:] {
		t = t.Add(time.Second)
		filter.Update(t, model.NewMeasurement(v))
		fmt.Printf("filtered value: %f\n", model.Value(filter.State()))
	}
}
```
