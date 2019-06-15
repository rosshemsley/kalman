package main

import (
	"log"
	"time"

	"github.com/fogleman/gg"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

const W = 600
const H = 600

type Observation struct {
	Time time.Time
	Point mat.Vector
}

func NewObservation(secondsOffset float64, x, y float64) Observation {
	return Observation {
		Point: mat.NewVecDense(2, []float64{x, y}),
		Time: time.Time{}.Add(time.Duration(secondsOffset * float64(time.Second))),
	}
}

var testData = []Observation{
	NewObservation(1, 0.06, 0.92),
	NewObservation(2, 0.06, 0.8),
	NewObservation(3, 0.08, 0.9),
	NewObservation(5, 0.08, 0.87),
	NewObservation(5.5, 0.16, 0.98),
	NewObservation(7, 0.15, 0.89),
	NewObservation(7.2, 0.19, 0.92),
	NewObservation(7.3, 0.18, 0.85),
	NewObservation(7.4, 0.26, 0.91),
	NewObservation(7.5, 0.24, 0.85),
	NewObservation(8, 0.27, 0.81),
	NewObservation(9, 0.28, 0.78),
	NewObservation(9.1, 0.35, 0.84),
	NewObservation(9.5, 0.35, 0.8),
	NewObservation(10, 0.46, 0.87),
	NewObservation(12, 0.44, 0.82),
	NewObservation(13, 0.44, 0.78),
	NewObservation(14, 0.5, 0.8),
	NewObservation(14.6, 0.49, 0.74),
	NewObservation(14.7, 0.7, 0.9),
	NewObservation(15, 0.57, 0.77),
	NewObservation(15.2, 0.61, 0.78),
	NewObservation(16, 0.6, 0.72),
	NewObservation(17, 0.67, 0.72),
	NewObservation(18, 0.62, 0.63),
	NewObservation(19, 0.66, 0.66),
	NewObservation(20.1, 0.75, 0.68),
	NewObservation(20.2, 0.73, 0.63),
	NewObservation(20.3, 0.76, 0.63),
	NewObservation(20.4, 0.75, 0.61),
	NewObservation(20.6, 0.94, 0.61),
	NewObservation(20.9, 0.79, 0.56),
	NewObservation(21, 0.82, 0.55),
	NewObservation(21.3, 0.73, 0.54),
	NewObservation(21.6, 0.8, 0.5),
	NewObservation(22, 0.81, 0.48),
	NewObservation(22.4, 0.63, 0.5),
	NewObservation(22.7, 0.66, 0.43),
	NewObservation(23, 0.6, 0.44),
	NewObservation(24, 0.65, 0.35),
	NewObservation(25, 0.57, 0.33),
	NewObservation(25.2, 0.68, 0.27),
	NewObservation(26, 0.48, 0.32),
	NewObservation(26.3, 0.47, 0.22),
	NewObservation(27, 0.39, 0.3),
	NewObservation(27.8, 0.39, 0.25),
	NewObservation(27.9, 0.38, 0.2),
	NewObservation(28, 0.35, 0.39),
	NewObservation(29, 0.32, 0.21),
	NewObservation(29.2, 0.32, 0.3),
	NewObservation(29.5, 0.29, 0.22),
	NewObservation(29.6, 0.25, 0.38),
	NewObservation(29.9, 0.23, 0.21),
	NewObservation(31, 0.15, 0.22),
	NewObservation(32, 0.18, 0.19),
	NewObservation(33, 0.12, 0.09),
	NewObservation(34, 0.17, 0.1),
	NewObservation(35, 0.17, 0.05),
	NewObservation(36, 0.26, 0.15),
	NewObservation(37, 0.25, 0.06),
	NewObservation(39, 0.29, 0.03),
	NewObservation(42, 0.35, 0.16),
	NewObservation(43, 0.35, 0.05),
	NewObservation(43.9, 0.4, 0.1),
	NewObservation(44.3, 0.42, 0.06),
	NewObservation(45, 0.55, 0.15),
	NewObservation(46, 0.55, 0.1),
	NewObservation(47, 0.64, 0.18),
	NewObservation(48, 0.65, 0.09),
	NewObservation(49, 0.69, 0.14),
	NewObservation(50, 0.71, 0.05),
	NewObservation(60, 0.76, 0.06),
	NewObservation(62, 0.8, 0.17),
	NewObservation(63, 0.8, 0.1),
	NewObservation(66, 0.87, 0.16),
	NewObservation(67, 0.91, 0.23),
	NewObservation(68, 0.86, 0.29),
	NewObservation(69, 0.98, 0.29),
	NewObservation(70, 0.95, 0.38),
}

// These numbers control how much smoothing the model does.
const observationNoise = 0.1  // entries for the diagonal of R_k
const initialVariance = 0.01  // entries for the diagonal of P_0
const processVariance = 0.005 // entries for the diagonal of Q_k

func main() {
	dc := gg.NewContext(W, H)
	dc.SetRGB(1, 1, 1)
	dc.Clear()

	model := models.NewConstantVelocityModel(testData[0].Time, testData[0].Point, models.ConstantVelocityModelConfig{
		InitialVariance: observationNoise,
		ProcessVariance: processVariance,
	})

	noisyTrajectory := extractTrajectory(testData)

	filteredTrajectory, err := kalmanFilter(model, testData)
	if err != nil {
		log.Fatalf("failed to run filter: %v", err)
	}

	smoothedTrajectory, err := kalmanSmoother(model, testData)
	if err != nil {
		log.Fatalf("failed to run smoother: %v", err)
	}

	dc.SetRGB(0, 1, 0)
	drawTrajectory(dc, noisyTrajectory)
	dc.SetRGB(1, 0, 0)
	drawTrajectory(dc, smoothedTrajectory)
	dc.SetRGB(0, 0, 1)
	drawTrajectory(dc, filteredTrajectory)

	err = dc.SavePNG("plot.png")
	if err != nil{ 
		log.Fatalf("failed to write png: %v", err)
	}
}

func kalmanFilter(model *models.ConstantVelocityModel, observations []Observation) ([]mat.Vector, error) {
	result := make([]mat.Vector, len(observations))	
	filter := kalman.NewKalmanFilter(model)

	for i, obs := range observations {
		err := filter.Update(obs.Time, model.NewPositionMeasurement(obs.Point, observationNoise))
		if err != nil {
			return nil, err
		}

		result[i] = model.Position(filter.State())
	}

	return result, nil
}

func kalmanSmoother(model *models.ConstantVelocityModel, observations []Observation) ([]mat.Vector, error) {
	mm := make([]*kalman.MeasurementAtTime, len(observations))
	for i, obs := range observations {
		mm[i] = kalman.NewMeasurementAtTime(obs.Time, model.NewPositionMeasurement(obs.Point, observationNoise))
	}

	states, err := kalman.NewKalmanSmoother(model).Smooth(mm...)
	if err != nil {
		return nil, err
	}

	result := make([]mat.Vector, len(states))
	for i, s := range states {
		result[i] = model.Position(s.State)
	}

	return result, nil
}

func extractTrajectory(observations []Observation) []mat.Vector {
	var result []mat.Vector
	for _, obs := range observations {
		result = append(result, obs.Point)
	}
	return result
}

func drawTrajectory(dc *gg.Context, t []mat.Vector) {
	for i, p := range t {
		if i == 0 {
			dc.MoveTo(p.AtVec(0)*W, p.AtVec(1)*H)
		} else {
			dc.LineTo(p.AtVec(0)*W, p.AtVec(1)*H)
		}
	}
	dc.Stroke()
}

func point(x, y float64) mat.Vector {
	return mat.NewVecDense(2, []float64{x, y})
}
