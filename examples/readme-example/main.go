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
