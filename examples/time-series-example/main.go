package main

import (
	"log"
	"math/rand"
	"time"

	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/plotutil"
	"gonum.org/v1/plot/vg"
)

const processVariance = 0.01
const initialVariance = 2.0
const observationVariance = 2.0

func main() {
	noisy, original := generateValues(20)
	
	model := models.NewSimpleModel(time.Time{}, noisy[0], models.SimpleModelConfig{
		InitialVariance:     initialVariance,
		ProcessVariance:     processVariance,
		ObservationVariance: observationVariance,
	})

	smoothed, err := kalmanSmoother(model, noisy)
	if err != nil {
		log.Fatalf("failed to smooth: %v", err)
	}

	filtered, err := kalmanFilter(model, noisy)
	if err != nil {
		log.Fatalf("failed to filter: %v", err)
	}


	var originalXYs, noisyXYs, filteredXYs, smoothedXYs plotter.XYs
	for i := 0; i<len(noisy); i++ {
		filteredXYs = append(filteredXYs, plotter.XY{
			X: float64(i),
			Y: filtered[i],
		})
		noisyXYs = append(noisyXYs, plotter.XY{
			X: float64(i),
			Y: noisy[i],
		})
		smoothedXYs = append(smoothedXYs, plotter.XY{
			X: float64(i),
			Y: smoothed[i],
		})
		originalXYs = append(originalXYs, plotter.XY{
			X: float64(i),
			Y: original[i],
		})
	}

	err = writePlotPNG("plot.png", "Kalman Filtered Time Series", "time since start/s", "",
		plottableValues{
			label:  "original",
			values: originalXYs,
		},
		plottableValues{
			label:  "noisy",
			values: noisyXYs,
		},
		plottableValues{
			label:  "filtered",
			values: filteredXYs,
		},
		plottableValues{
			label:  "smoothed",
			values: smoothedXYs,
		},
	)
	if err != nil {
		log.Fatalf("failed: %v", err)
	}
}

func kalmanFilter(model *models.SimpleModel, values []float64) ([]float64, error) {
	filter := kalman.NewKalmanFilter(model)

	var t time.Time
	result := make([]float64, len(values))

	for i, v := range values {
		t = t.Add(time.Second)

		err := filter.Update(t, model.NewMeasurement(v))
		if err != nil {
			return nil, err
		}

		result[i] = model.Value(filter.State())
	}

	return result, nil
}

func kalmanSmoother(model *models.SimpleModel, values []float64) ([]float64, error) {
	var t time.Time

	mm := make([]*kalman.MeasurementAtTime, len(values))
	for i, v := range values {
		t = t.Add(time.Second)
		mm[i] = kalman.NewMeasurementAtTime(t, model.NewMeasurement(v))
	}

	states, err := kalman.NewKalmanSmoother(model).Smooth(mm...)
	if err != nil {
		return nil, err
	}

	result := make([]float64, len(values))
	for i, s := range states {
		result[i] = model.Value(s.State)
	}
	
	return result, nil
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

	var xNoisy, xTruth float64

	for i := 0; i < n; i++ {
		delta := float64(i) * 0.001
		noise := observationVariance * rand.NormFloat64()

		xTruth += delta
		xNoisy = xTruth + noise

		noisy = append(noisy, xNoisy)
		truth = append(truth, xTruth)
	}

	return noisy, truth
}
