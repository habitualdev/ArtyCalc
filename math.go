package main

import (
	"bytes"
	"errors"
	"fmt"
	"fyne.io/fyne/v2"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"image"
	"image/png"
	"math"
	"strconv"
)

func CalcAngleHigh(initHeight, finalHeight, distance, velocity float64, drag bool) (float64, float64) {
	delta := finalHeight - initHeight
	tanThetaHigh := ((velocity * velocity) + math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)

	angle := math.Atan(tanThetaHigh) * r2m

	if !drag {
		_, tof, _ := CalculateForAngle(delta, angle, velocity, drag)

		return angle, tof
	}

	muz = velocity
	dist, tof := 0.0, 0.0
	for numberOfAttempts := 0.0; angle-numberOfAttempts > 800; numberOfAttempts++ {
		testAngle = angle - numberOfAttempts
		dist, tof, lastVectorPlotHigh = CalculateForAngle(delta, testAngle, muz, drag)
		currentError := distance - dist
		if math.Abs(currentError) < 2 {
			return testAngle, tof
		}
		if currentError < 0 {
			return testAngle, tof
		}

	}
	return 0, 0
}

func CalcAngleLow(initHeight, finalHeight, distance, velocity float64, drag bool) (float64, float64) {
	delta := finalHeight - initHeight

	coefComp := math.Sqrt(velocity*velocity*velocity*velocity - (g * (g*(distance*distance) + (2*delta)*(velocity*velocity))))
	if coefComp == math.NaN() {
		return 0, 0
	}

	tanThetaLow := ((velocity * velocity) - coefComp) / (g * distance)

	angle := math.Atan(tanThetaLow) * r2m
	if !drag {
		_, tof, _ := CalculateForAngle(delta, angle, velocity, drag)
		return angle, tof
	}
	muz = velocity
	dist, tof := 0.0, 0.0
	for numberOfAttempts := 0.0; numberOfAttempts+angle < 800; numberOfAttempts++ {
		testAngle = angle + numberOfAttempts
		dist, tof, lastVectorPlotLow = CalculateForAngle(delta, testAngle, muz, drag)
		currentError := distance - dist
		if math.Abs(currentError) < 2 {
			return testAngle, tof
		}
		if currentError < 0 {
			return testAngle, tof
		}
	}
	return 0, 0
}

func TimeOfFlight(velocity, theta, startHeight, endHeight float64) float64 {
	a2 := -g * math.Sin(theta) * math.Sin(theta) / 2.0
	b := velocity * math.Sin(theta)
	c := startHeight - endHeight
	n := fyne.NewNotification("TOF", fmt.Sprintf("a: %f b: %f c: %f", a, b, c))
	a.SendNotification(n)
	// Use the quadratic formula to solve for time (t):
	// at^2 + bt + c = 0
	determinant := b*b - 4*a2*c
	if determinant < 0 {
		// No real solutions exist (projectile doesn't reach the target height)
		return -1
	}

	t1 := (-b + math.Sqrt(determinant)) / (2 * a2)
	t2 := (-b - math.Sqrt(determinant)) / (2 * a2)

	// Choose the positive time solution
	if t1 > 0 {
		return t1
	} else if t2 > 0 {
		return t2
	} else {
		// No valid time of flight
		return -1
	}
}

func LinearSheaf(gunHeight, targetHeight, origDistance, shiftDistance, shiftAzimuth, azimuth, velocity float64, drag bool) string {
	xo := origDistance * math.Cos(azimuth/r2m)

	yo := origDistance * math.Sin(azimuth/r2m)

	xs := shiftDistance * math.Cos(shiftAzimuth/r2m)

	ys := shiftDistance * math.Sin(shiftAzimuth/r2m)

	xf := xo + xs
	yf := yo + ys

	newDistance := math.Sqrt((xf * xf) + (yf * yf))

	tanSelect := 0

	switch {
	case azimuth < 1600:
		tanSelect = 1
	case azimuth < 3200:
		tanSelect = 2
	case azimuth < 4800:
		tanSelect = 3
	case azimuth < 6400:
		tanSelect = 4
	}

	newAz := 0

	switch {
	case tanSelect == 3 || tanSelect == 2:
		newAz = int(math.Atan(yf/xf)*r2m) + 3200
	case tanSelect == 1 || tanSelect == 4:
		newAz = int(math.Atan(yf/xf) * r2m)
	}

	if newAz < 0 {
		newAz = newAz + 6400
	}

	low, _ := CalcAngleLow(gunHeight, targetHeight, newDistance, velocity, drag)
	high, _ := CalcAngleHigh(gunHeight, targetHeight, newDistance, velocity, drag)

	return fmt.Sprintf("Azimuth: %d | HA: %d | LA: %d", newAz, int(high), int(low))
}

func OTLAdjust(gunHeight, targetHeight, origDistance, shiftDistanceY, shiftDistanceX, otlAzimuth, azimuth, velocity float64, directionY string, directionX string, drag bool) string {
	adjustAzimuth := otlAzimuth
	adjustmentType := directionX + directionY //This concats the strings, not the actual meters

	// If any of these are 0, then there is an easier way of calculating this
	if shiftDistanceY != 0 && shiftDistanceX != 0 {
		deflection := math.Atan(shiftDistanceY/shiftDistanceX) * r2m
		switch adjustmentType {
		case "LeftAdd":
			adjustAzimuth = adjustAzimuth - ( 1600 - deflection)
			if adjustAzimuth < 0 {
				adjustAzimuth = adjustAzimuth + 6400
			}
		case "LeftDrop":
			adjustAzimuth = adjustAzimuth - ( 1600 + deflection)
			if adjustAzimuth < 0 {
				adjustAzimuth = adjustAzimuth + 6400
			}
		case "RightAdd":
			adjustAzimuth = adjustAzimuth + ( 1600 - deflection)
			if adjustAzimuth > 0 {
				adjustAzimuth = adjustAzimuth - 6400
			}
		case "RightDrop":
			adjustAzimuth = adjustAzimuth + ( 1600 + deflection)
			if adjustAzimuth > 0 {
				adjustAzimuth = adjustAzimuth - 6400
			}
		}
	}
	
	if shiftDistanceY == 0	{

		switch directionX {
		case "Right":
			adjustAzimuth = adjustAzimuth + 1600
			if adjustAzimuth > 0 {
				adjustAzimuth = adjustAzimuth - 6400
			}
		case "Left":
			adjustAzimuth = adjustAzimuth - 1600
			if adjustAzimuth < 0 {
				adjustAzimuth = adjustAzimuth + 6400
			}
		}
	}
	
	if shiftDistanceX == 0 {
		switch directionY {
			case "Drop":
				adjustAzimuth = adjustAzimuth - 3200
				if adjustAzimuth < 0 {
					adjustAzimuth = adjustAzimuth + 6400
				}
			default:
				break
		}
	}
	
	shiftDistance := math.Sqrt(math.Pow(shiftDistanceY, 2) + math.Pow(shiftDistanceX, 2))


	return LinearSheaf(gunHeight, targetHeight, origDistance, shiftDistance, adjustAzimuth, azimuth, velocity, drag)
}

func BoxSheaf(width, gunHeight, targetHeight, origDistance, shiftDistance, azimuth, velocity float64, drag bool) []string {
	shots := []string{"Impacts work from Top Left (original point) to Bottom Right"}
	shotsPerSide := width / shiftDistance

	rightAz := azimuth + 1600
	if rightAz > 6400 {
		rightAz = rightAz - 6400
	}

	for i := 0.0; i <= shotsPerSide; i++ {
		for j := 0.0; j <= shotsPerSide; j++ {
			shot := LinearSheaf(gunHeight, targetHeight, origDistance-(i*shiftDistance), j*shiftDistance, rightAz, azimuth, velocity, drag)
			shots = append(shots, shot)
		}
	}

	return shots
}

func DrawARect(originalPoint string, width, height, spread, orientation float64) []string {
	shots := []string{"Impacts work from Top Left (original point) to Bottom Right"}

	numShotsX := width / spread
	numShotsY := height / spread

	rightOrientation := orientation + 1600

	if rightOrientation >= 6400 {
		rightOrientation = rightOrientation - 6400
	}
	
	for i := 0.0; i <= numShotsX; i++ {

		startPoint, _ := PolarToGrid(strconv.Itoa(int(spread*i)), strconv.Itoa(int(rightOrientation)), originalPoint)
		for j := 0.0; j <= numShotsY; j++ {
			newGrid, _ := PolarToGrid(strconv.Itoa(int(spread*j)), strconv.Itoa(int(orientation)), startPoint)
			shots = append(shots, newGrid)
		}
	}
	return shots
}

func PolarToGrid(distance, azimuth, gunPos string) (string, error) {

	distanceFloat, err := strconv.ParseFloat(distance, 64)

	az, err := strconv.Atoi(azimuth)
	if err != nil {
		return "", err
	}
	sinDelta := math.Sin(float64(az)/r2m) * distanceFloat
	cosDelta := math.Cos(float64(az)/r2m) * distanceFloat

	switch {
	case len(gunPos) == 6:
		easting := gunPos[:3]
		northing := gunPos[3:]
		gunPos = easting + "00" + northing + "00"
	case len(gunPos) == 8:
		easting := gunPos[:4]
		northing := gunPos[4:]
		gunPos = easting + "0" + northing + "0"
	case len(gunPos) == 10:
		break
	default:
		return "", errors.New("Check your gun position grid.")
	}

	quadrant := 0

	switch {
	case az < 1600:
		quadrant = 1
	case az < 3200:
		quadrant = 2
	case az < 4800:
		quadrant = 3
	case az < 6400:
		quadrant = 4
	}

	eastingGun := gunPos[:5]
	northingGun := gunPos[5:]

	eastingGunInt, err := strconv.Atoi(eastingGun)
	northingGunInt, err := strconv.Atoi(northingGun)

	targetNorthing := 0
	targetEasting := 0

	switch {
	case quadrant == 1:
		targetNorthing = northingGunInt + int(cosDelta)
		targetEasting = eastingGunInt + int(sinDelta)
	case quadrant == 2:
		targetNorthing = northingGunInt + int(cosDelta)
		targetEasting = eastingGunInt + int(sinDelta)
	case quadrant == 3:
		targetNorthing = northingGunInt + int(cosDelta)
		targetEasting = eastingGunInt + int(sinDelta)
	case quadrant == 4:
		targetNorthing = northingGunInt + int(cosDelta)
		targetEasting = eastingGunInt + int(sinDelta)
	}

	targetEastingString := fmt.Sprintf("%d", targetEasting)
	if len(targetEastingString) < 5 {
		for len(targetEastingString) != 5 {
			targetEastingString = "0" + targetEastingString
		}
	}

	targetNorthingString := fmt.Sprintf("%d", targetNorthing)
	if len(targetNorthingString) < 5 {
		for len(targetNorthingString) != 5 {
			targetNorthingString = "0" + targetNorthingString
		}
	}

	targetGrid := targetEastingString + targetNorthingString

	return targetGrid, nil
}

func CalcDistance(gunPos, targetPos string) (float64, error) {
	goodGrids := false
	switch {
	case len(gunPos) == 6:
		easting := gunPos[:3]
		northing := gunPos[3:]
		gunPos = easting + "00" + northing + "00"
		goodGrids = true
	case len(gunPos) == 8:
		easting := gunPos[:4]
		northing := gunPos[4:]
		gunPos = easting + "0" + northing + "0"
		goodGrids = true
	case len(gunPos) == 10:
		goodGrids = true
	default:
		return math.NaN(), errors.New("Check your gun position grid.")
	}
	switch {
	case len(targetPos) == 6:
		easting := targetPos[:3]
		northing := targetPos[3:]
		targetPos = easting + "00" + northing + "00"
		goodGrids = true
	case len(targetPos) == 8:
		easting := targetPos[:4]
		northing := targetPos[4:]
		targetPos = easting + "0" + northing + "0"
		goodGrids = true
	case len(targetPos) == 10:
		goodGrids = true
	default:
		return math.NaN(), errors.New("Request CFF Readback. Bad Target Grid.")
	}
	if !goodGrids {
		return math.NaN(), errors.New("Request CFF Readback.")
	}

	eastingTarget := targetPos[:5]
	northingTarget := targetPos[5:]
	eastingGun := gunPos[:5]
	northingGun := gunPos[5:]

	eastingTargetInt, err := strconv.Atoi(eastingTarget)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in target grid")
	}
	northingTargetInt, err := strconv.Atoi(northingTarget)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in target grid")
	}
	eastingGunInt, err := strconv.Atoi(eastingGun)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in gun position grid")
	}
	northingGunInt, err := strconv.Atoi(northingGun)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in gun position grid")
	}

	deltaEasting := eastingTargetInt - eastingGunInt
	deltaNorthing := northingTargetInt - northingGunInt

	finalDelta := math.Sqrt(float64((deltaNorthing * deltaNorthing) + (deltaEasting * deltaEasting)))

	return finalDelta, nil

}

func CalcAzimuth(gunPos, targetPos string) (float64, error) {
	goodGrids := false
	switch {
	case len(gunPos) == 6:
		easting := gunPos[:3]
		northing := gunPos[3:]
		gunPos = easting + "00" + northing + "00"
		goodGrids = true
	case len(gunPos) == 8:
		easting := gunPos[:4]
		northing := gunPos[4:]
		gunPos = easting + "0" + northing + "0"
		goodGrids = true
	case len(gunPos) == 10:
		goodGrids = true
	default:
		return math.NaN(), errors.New("check your gun position grid")
	}
	switch {
	case len(targetPos) == 6:
		easting := targetPos[:3]
		northing := targetPos[3:]
		targetPos = easting + "00" + northing + "00"
		goodGrids = true
	case len(targetPos) == 8:
		easting := targetPos[:4]
		northing := targetPos[4:]
		targetPos = easting + "0" + northing + "0"
		goodGrids = true
	case len(targetPos) == 10:
		goodGrids = true
	default:
		return math.NaN(), errors.New("request CFF Readback. Bad Target Grid")
	}
	if !goodGrids {
		return math.NaN(), errors.New("request CFF Readback")
	}

	eastingTarget := targetPos[:5]
	northingTarget := targetPos[5:]
	eastingGun := gunPos[:5]
	northingGun := gunPos[5:]

	eastingTargetInt, err := strconv.Atoi(eastingTarget)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in target grid")
	}
	northingTargetInt, err := strconv.Atoi(northingTarget)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in target grid")
	}
	eastingGunInt, err := strconv.Atoi(eastingGun)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in gun position grid")
	}
	northingGunInt, err := strconv.Atoi(northingGun)
	if err != nil {
		return math.NaN(), errors.New("Non-Number in gun position grid")
	}

	quadrant := 0

	switch {
	case eastingTargetInt > eastingGunInt && northingTargetInt > northingGunInt:
		quadrant = 1
	case eastingTargetInt > eastingGunInt && northingGunInt > northingTargetInt:
		quadrant = 2
	case eastingGunInt > eastingTargetInt && northingGunInt > northingTargetInt:
		quadrant = 3
	case northingTargetInt > northingGunInt && eastingTargetInt < eastingGunInt:
		quadrant = 4
	}

	deltaEasting := eastingTargetInt - eastingGunInt
	deltaNorthing := northingTargetInt - northingGunInt

	azMils := 0.0
	if quadrant == 3 || quadrant == 2 {
		azMils = math.Atan(float64(deltaEasting)/float64(deltaNorthing)) * r2m
		azMils = azMils + 3200
	} else {
		azMils = math.Atan(float64(deltaEasting)/float64(deltaNorthing)) * r2m
	}

	if azMils < 0 {
		azMils = azMils + 6400
	}
	if azMils > 6400 {
		azMils = azMils - 6400
	}

	return azMils, nil

}

func GraphShotNoDrag(distance, azimuth, muzzleVelocity, gunAltitude, targetAltitude float64, lowHigh string) image.Image {
	if azimuth < 0 || azimuth > 1600 || math.IsNaN(azimuth) {
		return image.NewRGBA(image.Rect(0, 0, 100, 100))
	}

	p := plot.New()
	p.Title.Text = "Ballistic Solution - " + lowHigh
	p.X.Label.Text = "Distance"
	p.Y.Label.Text = "Elevation"

	p.X.Min = 0
	p.X.Max = distance
	p.Y.Max = distance
	p.Y.Min = 0

	p.Y.AutoRescale = true

	velocityY := muzzleVelocity * math.Sin(azimuth/r2m)
	velocityX := muzzleVelocity * math.Cos(azimuth/r2m)

	timeTick := 1.0 / 60.0
	t := 0.0
	BallisticPath := []BallisticPoint{}

	apex := 0.0

	for {
		newX := velocityX * t
		newY := velocityY*t + (-9.8*t*t)/2
		newY = newY + gunAltitude
		if newY > apex {
			apex = newY
		}
		BallisticPath = append(BallisticPath, BallisticPoint{X: newX, Y: newY})
		t = t + timeTick
		if newX > distance {
			break
		}
		if t > 300 {
			break
		}
	}

	pts := make(plotter.XYs, len(BallisticPath))
	for i, p := range BallisticPath {
		pts[i].X = p.X
		pts[i].Y = p.Y
	}

	plotBallistics, err := plotter.NewLine(pts)

	if err != nil {
		panic(err)
	}
	p.Add(plotBallistics)

	apexString := fmt.Sprintf("%.2f", apex)

	p.Legend.Add("Apex - "+apexString, plotBallistics)
	p.Legend.ThumbnailWidth = 0

	wt, err := p.WriterTo(4*vg.Inch, 4*vg.Inch, "png")
	if err != nil {
		panic(err)
	}
	buf := new(bytes.Buffer)
	_, err = wt.WriteTo(buf)
	if err != nil {
		panic(err)
	}
	img, err := png.Decode(buf)

	return img
}

func GraphShotDrag(dataPoints plotter.XYs, distance float64, lowHigh string) image.Image {
	dataGraph, _ := plotter.NewLine(dataPoints)
	maxOrd := 0.0
	for _, y := range dataPoints {
		if y.Y > maxOrd {
			maxOrd = y.Y
		}
	}

	p := plot.New()
	p.Title.Text = "Ballistic Solution - " + lowHigh
	p.X.Label.Text = "Distance"
	p.Y.Label.Text = "Elevation"

	p.X.Min = 0
	p.X.Max = distance
	p.Y.Max = distance
	p.Y.Min = 0

	p.Y.AutoRescale = true

	p.Add(dataGraph)

	apexString := fmt.Sprintf("%.2f", maxOrd)

	p.Legend.Add("Apex - "+apexString, dataGraph)
	p.Legend.ThumbnailWidth = 0
	wt, err := p.WriterTo(4*vg.Inch, 4*vg.Inch, "png")
	if err != nil {
		panic(err)
	}
	buf := new(bytes.Buffer)
	_, err = wt.WriteTo(buf)
	if err != nil {
		panic(err)
	}
	img, err := png.Decode(buf)

	return img

}
