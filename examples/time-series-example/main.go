package main

import (
	"log"
	"math"
	"math/rand"
	"time"

	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/plotutil"
	"gonum.org/v1/plot/vg"
)

func main() {
	noisy, original := generateValues(100)
	startTime := time.Time{}
	model := models.NewSimpleModel(startTime, noisy[0], models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     1.0,
		ObservationVariance: 2.0,
	})
	filter := kalman.NewKalmanFilter(model)

	var hiddenXYs, originalXYs, filteredXYs plotter.XYs

	t := startTime
	for i, v := range noisy[1:] {
		t = t.Add(time.Second)
		filter.Update(t, model.NewMeasurement(v))

		filteredXYs = append(filteredXYs, plotter.XY{
			X: float64(i),
			Y: model.Value(filter.State()),
		})
		originalXYs = append(originalXYs, plotter.XY{
			X: float64(i),
			Y: v,
		})
		hiddenXYs = append(hiddenXYs, plotter.XY{
			X: float64(i),
			Y: original[i],
		})
	}

	err := writePlotPNG("plot.png", "Kalman Filtered Time Series", "time since start/s", "",
		plottableValues{
			label:  "original",
			values: originalXYs,
		},
		plottableValues{
			label:  "filtered",
			values: filteredXYs,
		},
		plottableValues{
			label:  "true",
			values: hiddenXYs,
		},
	)
	if err != nil {
		log.Fatalf("failed: %v", err)
	}
}

type plottableValues struct {
	label  string
	values plotter.XYs
}

func writePlotPNG(path, title, xLabel, yLabel string, lines ...plottableValues) error {
	p, err := plot.New()
	if err != nil {
		return err
	}

	p.Title.Text = title
	p.X.Label.Text = xLabel
	p.Y.Label.Text = yLabel

	var pp []interface{}
	for _, l := range lines {
		pp = append(pp, l.label, l.values)
	}

	err = plotutil.AddLines(p, pp...)
	if err != nil {
		return err
	}

	return p.Save(5*vg.Inch, 5*vg.Inch, path)
}

func generateValues(n int) (noisy, truth []float64) {
	noisy = make([]float64, 0)
	truth = make([]float64, 0)

	variance := 4.0
	var lastNoisy, lastTruth float64

	for i := 0; i < n; i++ {
		delta := 5*math.Sin(math.Pi*float64(i)/7) + 3*math.Sin(math.Pi*float64(i)/17.0)
		noise := variance * rand.NormFloat64()

		lastNoisy += delta + noise
		lastTruth += delta

		noisy = append(noisy, lastNoisy)
		truth = append(truth, lastTruth)
	}

	return noisy, truth
}
