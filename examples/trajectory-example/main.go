package main

import (
	"log"
	"time"

	"github.com/fogleman/gg"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

const W = 1000
const H = 1000

var noisyTrajectory = []mat.Vector{
	point(0.06, 0.92),
	point(0.06, 0.8),
	point(0.08, 0.9),
	point(0.08, 0.87),
	point(0.16, 0.98),
	point(0.15, 0.89),
	point(0.19, 0.92),
	point(0.18, 0.85),
	point(0.26, 0.91),
	point(0.24, 0.85),
	point(0.27, 0.81),
	point(0.28, 0.78),
	point(0.35, 0.84),
	point(0.35, 0.8),
	point(0.46, 0.87),
	point(0.44, 0.82),
	point(0.44, 0.78),
	point(0.5, 0.8),
	point(0.49, 0.74),
	point(0.7, 0.9),
	point(0.57, 0.77),
	point(0.61, 0.78),
	point(0.6, 0.72),
	point(0.67, 0.72),
	point(0.62, 0.63),
	point(0.66, 0.66),
	point(0.75, 0.68),
	point(0.73, 0.63),
	point(0.76, 0.63),
	point(0.75, 0.61),
	point(0.94, 0.61),
	point(0.79, 0.56),
	point(0.82, 0.55),
	point(0.73, 0.54),
	point(0.8, 0.5),
	point(0.81, 0.48),
	point(0.63, 0.5),
	point(0.66, 0.43),
	point(0.6, 0.44),
	point(0.65, 0.35),
	point(0.57, 0.33),
	point(0.68, 0.27),
	point(0.48, 0.32),
	point(0.47, 0.22),
	point(0.39, 0.3),
	point(0.39, 0.25),
	point(0.38, 0.2),
	point(0.35, 0.39),
	point(0.32, 0.21),
	point(0.32, 0.3),
	point(0.29, 0.22),
	point(0.25, 0.38),
	point(0.23, 0.21),
	point(0.15, 0.22),
	point(0.18, 0.19),
	point(0.12, 0.09),
	point(0.17, 0.1),
	point(0.17, 0.05),
	point(0.26, 0.15),
	point(0.25, 0.06),
	point(0.29, 0.03),
	point(0.35, 0.16),
	point(0.35, 0.05),
	point(0.4, 0.1),
	point(0.42, 0.06),
	point(0.55, 0.15),
	point(0.55, 0.1),
	point(0.64, 0.18),
	point(0.65, 0.09),
	point(0.69, 0.14),
	point(0.71, 0.05),
	point(0.76, 0.06),
	point(0.8, 0.17),
	point(0.8, 0.1),
	point(0.87, 0.16),
	point(0.91, 0.23),
	point(0.86, 0.29),
	point(0.98, 0.29),
	point(0.95, 0.38),
}

func main() {
	dc := gg.NewContext(W, H)

	startTime := time.Time{}
	model := models.NewConstantVelocityModel(startTime, noisyTrajectory[0], models.ConstantVelocityModelConfig{
		InitialVariance: 0.01,
		ProcessVariance: 0.0005,
	})

	filter := kalman.NewKalmanFilter(model)

	var smoothedTrajectory []mat.Vector
	smoothedTrajectory = append(smoothedTrajectory, model.Position(filter.State()))

	for i, v := range noisyTrajectory[1:] {
		t := startTime.Add(time.Duration(i * int(time.Second)))

		filter.Update(t, model.NewPositionMeasurement(v, 0.1))
		smoothedTrajectory = append(smoothedTrajectory, model.Position(filter.State()))
	}

	var t time.Time
	var ms []kalman.MeasurementAtTime
	for _, v := range noisyTrajectory {
		t = t.Add(time.Second)
		ms = append(ms, kalman.MeasurementAtTime{
			Time:        t,
			Measurement: *model.NewPositionMeasurement(v, 0.1),
		})
	}
	smoother := kalman.NewKalmanSmoother(model)
	forwardsStates, _ := smoother.ComputeForwardsStateChanges(ms...)
	smoothedTrajectory = make([]mat.Vector, 0)

	for _, vs := range forwardsStates {
		smoothedTrajectory = append(smoothedTrajectory, vs.APoseterioriState)
	}

	smoothed, err := smoother.Smooth(ms...)
	if err != nil {
		panic(err)
	}

	log.Printf("len: %d, %d", len(smoothed), len(noisyTrajectory))
	var vs []mat.Vector
	for _, v := range smoothed {
		vs = append(vs, model.Position(v.State))
	}

	dc.SetRGB(0, 1, 0)
	drawTrajectory(dc, noisyTrajectory)
	dc.SetRGB(1, 0, 0)
	drawTrajectory(dc, smoothedTrajectory)
	dc.SetRGB(0, 0, 1)
	drawTrajectory(dc, vs)

	dc.SavePNG("plot.png")
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
