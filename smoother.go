package kalman

import (
	"time"
	"log"
	"fmt"

	"github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
)

type kalmanStateChange struct {
	// The transition used to advance the model from the previous
	// aPosteriori estimate to the current a Priori estimate.
	// T_k
	modelTransition mat.Matrix

	// State before measurement taken, x_{k|k-1}, P_{k|k-1}
	aPrioriState mat.Vector
	aPrioriCovariance mat.Matrix

	// State after measurement taken, x_{k|k}, P_{k|k}
	APoseterioriState mat.Vector
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

	ss, err := kf.ComputeForwardsStateChanges(measurements...)
	if err != nil {
		return nil, err
	}
	
	dims := ss[0].aPrioriState.Len()
	C := mat.NewDense(dims, dims, nil)
	aPosterioriCovarianceInv := mat.NewDense(dims, dims, nil)

	result := make([]models.State, n)
	result[n-1].State = ss[n-1].APoseterioriState
	result[n-1].Covariance = ss[n-1].aPosterioriCovariance

	x := mat.NewVecDense(dims, nil)
	P := mat.NewDense(dims, dims, nil)


	for i := n - 2 ; i >= 0; i-- {
		err = aPosterioriCovarianceInv.Inverse(ss[i+1].aPrioriCovariance)
		if err != nil {
			panic(err)
		}

		C.Product(
			ss[i].aPosterioriCovariance,
			ss[i+1].modelTransition.T(),
			aPosterioriCovarianceInv,
		)

		x.SubVec(result[i+1].State, ss[i+1].aPrioriState)
		x.MulVec(C, x)
		x.AddVec(ss[i].APoseterioriState, x)
		
		P.Sub(result[i+1].Covariance, ss[i+1].aPrioriCovariance)
		P.Product(C, P, C.T())
		P.Add(ss[i].aPosterioriCovariance, P)

		result[i].State = x
		result[i].Covariance = P
	}

	return result, nil
}

// ComputeForwardsStateChanges runs the regular KalmanFilter for the given measurements.
func (kf *KalmanSmoother)ComputeForwardsStateChanges(measurements ...MeasurementAtTime) ([]kalmanStateChange, error) {
	filter := NewKalmanFilter(kf.model)
	result := make([]kalmanStateChange, len(measurements))

	for i, m := range measurements {
		stateChange := &result[i]
		dt := m.Time.Sub(filter.Time())
		log.Printf("DT: %s", dt)

		stateChange.modelTransition = kf.model.Transition(dt)
		// prettyMat(stateChange.modelTransition)
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
		stateChange.APoseterioriState = filter.State()
		// fmt.Printf("STATE %d\n", i)
		// prettyVec(filter.State())

		stateChange.aPosterioriCovariance = filter.Covariance()

		// prettyMat(filter.Covariance())
		
	}

	return result, nil
}

func prettyMat(m mat.Matrix) {
	rows, cols := m.Dims()

	for r :=0; r != rows; r++ {
		for c :=0; c != cols; c++ {
			fmt.Printf("%.3f    ", m.At(r,c))
		}
		fmt.Printf("\n")
	}
	fmt.Printf("\n")
}

func prettyVec(v mat.Vector) {
	for r :=0; r != v.Len(); r++ {
		fmt.Printf("%.3f    ", v.AtVec(r))
	}
	fmt.Printf("\n")
}
