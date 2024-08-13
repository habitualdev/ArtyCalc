package main

import (
	_ "embed"
	"fmt"
	"gonum.org/v1/plot/plotter"
	"math"
)

//go:embed drag.json
var dragTable []byte

type DragTable map[string]float64

var dragTableJson = DragTable{}

const timeStep = 1.0 / 60

type vector3 struct {
	x float64
	y float64
}

func (v vector3) magnitude() float64 {
	return math.Sqrt(v.x*v.x + v.y*v.y)
}

func (v vector3) subtract(vector32 vector3) vector3 {
	newVector := vector3{}
	newVector.x = v.x - vector32.x
	newVector.y = v.y - vector32.y
	return newVector
}

func (v vector3) add(vector32 vector3) vector3 {
	newVector := vector3{}
	newVector.x = v.x + vector32.x
	newVector.y = v.y + vector32.y
	return newVector
}

func (v vector3) mul(vector32 vector3) vector3 {
	newVector := vector3{}
	newVector.x = v.x * vector32.x
	newVector.y = v.y * vector32.y
	return newVector
}

func (v vector3) divide(vector32 vector3) vector3 {
	newVector := vector3{}
	newVector.x = v.x / vector32.x
	newVector.y = v.y / vector32.y
	return newVector
}

func (v vector3) lerp(vector vector3, t float64) vector3 {
	a := vector3{
		x: v.x * t,
		y: v.y * t,
	}
	b := vector3{
		x: vector.x * (1.0 - t),
		y: vector.y * (1.0 - t),
	}
	return a.add(b)
}

func CalculateForAngle(delta, testAngle, muz float64, drag bool) (float64, float64, plotter.XYs) {
	newPlot := plotter.XYs{}
	trueDrag := 0.0
	if drag {
		trueDrag = dragTableJson[currentGunString]
	}

	k := -1 * trueDrag * 1
	temperature := 15.0
	currentPos := vector3{
		x: 0,
		y: 0,
	}

	lastPos := currentPos

	currentVelocity := vector3{
		x: muz * math.Cos(testAngle/r2m) * (((temperature+273.13)/288.13-1.0)/40.0 + 1.0),
		y: muz * math.Sin(testAngle/r2m) * (((temperature+273.13)/288.13-1.0)/40.0 + 1.0),
	}

	wind := vector3{
		x: 0,
		y: 0,
	}

	grav := vector3{
		x: 0,
		y: -9.8066,
	}

	currentTime := 0.0
	for currentVelocity.y > 0 || currentPos.y >= delta {
		lastPos = currentPos
		apparentWind := wind.subtract(currentVelocity)
		changeInVelocity := grav.add(apparentWind.mul(vector3{
			x: k * apparentWind.magnitude(),
			y: k * apparentWind.magnitude(),
		}))
		fmt.Println(changeInVelocity)
		currentVelocity = vector3{
			x: currentVelocity.x + (changeInVelocity.x * timeStep),
			y: currentVelocity.y + (changeInVelocity.y * timeStep),
		}
		currentPos = vector3{
			x: currentPos.x + (currentVelocity.x * timeStep),
			y: currentPos.y + (currentVelocity.y * timeStep),
		}
		currentTime += timeStep
		newPlot = append(newPlot, plotter.XY{
			X: currentPos.x,
			Y: currentPos.y,
		})
	}

	lastRatio := (delta - currentPos.y) / (lastPos.y - currentPos.y)

	return lastPos.lerp(currentPos, lastRatio).x, currentTime, newPlot

}
