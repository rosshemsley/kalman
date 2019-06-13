package kalman

import (
	"time"

	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

type kalmanStateChange struct {
	// The transition used to advance the model from the previous
	// aPosteriori estimate to the current a Priori estimate.
	modelTransition mat.Matrix

	// State before measurement taken
	aPrioriState mat.Vector
	aPrioriCovariance mat.Matrix

	// State after measurement taken
	aPosterioriState mat.Vector
	aPosterioriCovariance mat.Matrix
}

type KalmanSmoother struct {
	model models.LinearModel
}

func NewKalmanSmoother(model models.LinearModel) *KalmanSmoother {
	return &KalmanSmoother{
		model: model, 
	}
}

type MeasurementAtTime struct {
	models.Measurement
	Time time.Time
}

func (kf *KalmanSmoother)Smooth(measurements ...MeasurementAtTime) ([]models.State, error){
	n := len(measurements)
	if n == 0 {
		return make([]models.State, 0), nil
	}

	stateChanges, err := kf.computeForwardsStateChanges(measurements...)
	if err != nil {
		return nil, err
	}
	
	dims := stateChanges[0].aPrioriState.Len()
	C := mat.NewDense(dims, dims, nil)
	aPosterioriCovarianceInv := mat.NewDense(dims, dims, nil)

	result := make([]models.State, 0)
	result[n-1].State = stateChanges[n-1].aPosterioriState
	result[n-1].Covariance = stateChanges[n-1].aPosterioriCovariance

	x := mat.NewVecDense(dims, nil)
	P := mat.NewDense(dims, dims, nil)

	for i:= n -1 ; i >= 0; i-- {
		sc := &stateChanges[i]
		aPosterioriCovarianceInv.Inverse(sc.aPosterioriCovariance)

		C.Product(
			sc.aPrioriState,
			sc.modelTransition,
			aPosterioriCovarianceInv,
		)

		x.SubVec(result[i+1].State, sc.aPosterioriState)
		x.MulVec(C, x)
		x.AddVec(sc.aPrioriState, x)
		
		P.Sub(result[i+1].Covariance, sc.aPosterioriCovariance)
		P.Product(C, P, C.T())
		P.Add(sc.aPrioriCovariance, P)

		result[i].State = x
		result[i].Covariance = P
	}

	return result, nil
}

// computeForwardsStateChanges runs the regular KalmanFilter for the given measurements.
func (kf *KalmanSmoother)computeForwardsStateChanges(measurements ...MeasurementAtTime) ([]kalmanStateChange, error) {
	filter := NewKalmanFilter(kf.model)
	result := make([]kalmanStateChange, len(measurements))

	for i, m := range measurements {
		stateChange := &result[i]
		dt := m.Time.Sub(filter.Time())

		stateChange.modelTransition = kf.model.Transition(dt)
		err := filter.Predict(m.Time)
		if err != nil {
			return nil, err
		}

		stateChange.aPrioriState = filter.State()
		stateChange.aPrioriCovariance = filter.Covariance()

		err = filter.Update(m.Time, &m.Measurement)
		if err != nil {
			return nil, err
		}

		stateChange.aPosterioriState = filter.State()
		stateChange.aPosterioriCovariance = filter.Covariance()		
	}

	return result, nil
}
