package main

import (
	"bytes"
	_ "embed"
	"encoding/json"
	"errors"
	"fmt"
	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/dialog"
	"fyne.io/fyne/v2/theme"
	"fyne.io/fyne/v2/widget"
	"github.com/google/uuid"
	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
	"image"
	"image/png"
	"math"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"time"
)

//go:embed degrees_mills.png
var degreeMils []byte

//go:embed artillery.json
var artilleryJson []byte

//go:embed audio/splash_1.mp3
var splashSound1 []byte

//go:embed audio/splash_2.mp3
var splashSound2 []byte

//go:embed audio/solution_1.mp3
var solution1 []byte

//go:embed audio/solution_2.mp3
var solution2 []byte

//go:embed audio/splash_5_1.mp3
var splashInFive1 []byte

//go:embed audio/splash_5_2.mp3
var splashInFive2 []byte

const g float64 = 9.80665

const dragCoef float64 = -0.00006

const r2m float64 = 1018.591636

const addMarker = "/api/markers/add"

const deleteMarker = "/api/markers/delete"

var a fyne.App

var mute = false

var testAngle = 0.0

var muz = 0.0

var currentGunString string

var lastVectorPlotHigh plotter.XYs

var lastVectorPlotLow plotter.XYs

var habTacSync = false

type BallisticPoint struct {
	X float64
	Y float64
}

type Marker struct {
	Lat      string `json:"lat"`
	Lng      string `json:"lng"`
	Elv      string `json:"elv"`
	Title    string `json:"title"`
	Unit     string `json:"unit"`
	Map      string `json:"map"`
	Comments string `json:"comments"`
	Id       string `json:"id"`
}

func CalcAngleHigh(initHeight, finalHeight, distance, velocity float64, drag bool) (float64, float64) {
	delta := finalHeight - initHeight
	tanThetaHigh := ((velocity * velocity) + math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)

	angle := math.Atan(tanThetaHigh) * r2m

	if !drag {
		return angle, TimeOfFlight(velocity, angle/r2m)
	}

	muz = velocity
	dist, tof := 0.0, 0.0
	for numberOfAttempts := 0.0; angle-numberOfAttempts > 800; numberOfAttempts++ {
		testAngle = angle - numberOfAttempts
		dist, tof, lastVectorPlotHigh = CalculateForAngle(delta, testAngle, muz)
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
		return angle, TimeOfFlight(velocity, angle/r2m)
	}
	muz = velocity
	dist, tof := 0.0, 0.0
	for numberOfAttempts := 0.0; numberOfAttempts+angle < 800; numberOfAttempts++ {
		testAngle = angle + numberOfAttempts
		dist, tof, lastVectorPlotLow = CalculateForAngle(delta, testAngle, muz)
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

func TimeOfFlight(velocity, angle float64) float64 {
	tof := (2 * velocity * math.Sin(angle)) / g
	return tof
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

	for i := 0.0; i <= numShotsX; i++ {

		startPoint, _ := PolarToGrid(strconv.Itoa(int(spread*i)), strconv.Itoa(int(orientation+1600)), originalPoint)
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

func GetLatestNumbers() Guns {
	tempGuns := Guns{}
	resp, err := http.Get("https://raw.githubusercontent.com/habitualdev/ArtyCalc/main/artillery.json")
	if err != nil {
		err = json.Unmarshal(artilleryJson, &tempGuns)
		return tempGuns
	}
	defer resp.Body.Close()
	buf := []byte{}
	resp.Body.Read(buf)
	err = json.Unmarshal(buf, &tempGuns)
	if err != nil {
		err = json.Unmarshal(artilleryJson, &tempGuns)
		return tempGuns
	}
	return tempGuns
}

func GetLatestNumbersDrag() DragTable {
	artyDrag := DragTable{}
	resp, err := http.Get("https://raw.githubusercontent.com/habitualdev/ArtyCalc/main/drag.json")
	if err != nil {
		err = json.Unmarshal(dragTable, &artyDrag)
		return artyDrag
	}
	defer resp.Body.Close()
	buf := []byte{}
	resp.Body.Read(buf)
	err = json.Unmarshal(buf, &artyDrag)
	if err != nil {
		err = json.Unmarshal(dragTable, &artyDrag)
		return artyDrag
	}
	return artyDrag
}

var sessionUuid = ""

func main() {
	sessionUuid = uuid.New().String()
	lastCalcMission := FireMission{}
	var savedMissions FireMissions
	airResistanceBool := false
	guns := Guns{}

	curGun := Gun{}

	guns = GetLatestNumbers()
	dragTableJson = GetLatestNumbersDrag()

	a = app.New()
	a.Settings().SetTheme(theme.DarkTheme())
	w := a.NewWindow("CFF Calculator")
	azimuthBoxGrid := widget.NewLabel("")
	gunElevationPolar := widget.NewLabel("")
	gunElevationGrid := widget.NewLabel("")
	airResistance := widget.NewCheck("Air Resistance", func(checked bool) {
		airResistanceBool = checked
	})
	muteButton := widget.NewButton("Toggle Sound", func() {
		if mute {
			mute = false
		} else {
			mute = true
		}
	})

	habTacLogin := widget.NewCheck("HabTac Sync", func(checked bool) {
		habTacSync = checked
	})

	timeOfFlight := widget.NewLabel("TOF: ")

	azimuthBoxPolar := widget.NewEntry()
	distancePolar := widget.NewEntry()

	gunGrid := widget.NewEntry()
	gunAltitude := widget.NewEntry()

	targetGrid := widget.NewEntry()
	targetAltitude := widget.NewEntry()

	targetName := widget.NewEntry()
	targetName.SetPlaceHolder("TRP Name")

	saveMissionButtonGrid := widget.NewButton("SaveMission", func() {
		lastCalcMission.Name = targetName.Text
		savedMissions = append(savedMissions, lastCalcMission)

	})

	gunList := []string{}
	for gun, _ := range guns {
		gunList = append(gunList, gun)
	}

	curGunCharges := []string{}
	for charge, _ := range curGun {
		curGunCharges = append(curGunCharges, charge)
	}

	chargeSelection := widget.NewSelect(curGunCharges, nil)

	gunSelection := widget.NewSelect(gunList, func(selectedGun string) {
		curGun = guns[selectedGun]
		currentGunString = selectedGun
		curGunCharges = []string{}
		for charge, _ := range curGun {
			curGunCharges = append(curGunCharges, charge)
		}
		chargeSelection.Options = curGunCharges
	})

	gunSelectRow := container.NewHBox(gunSelection, chargeSelection, habTacLogin)

	highAngleImage := canvas.NewImageFromImage(image.NewRGBA(image.Rect(0, 0, 100, 100)))
	lowAngleImage := canvas.NewImageFromImage(image.NewRGBA(image.Rect(0, 0, 100, 100)))

	calculateMissionGrid := widget.NewButton("Calculate Mission", func() {
		dist, err := CalcDistance(gunGrid.Text, targetGrid.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		gunAlt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		targetAlt, err := strconv.Atoi(targetAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		if curGun == nil {
			a.SendNotification(fyne.NewNotification("Error", "No gun selected"))
			return
		}

		if chargeSelection.Selected == "" {
			a.SendNotification(fyne.NewNotification("Error", "No charge selected"))
			return
		}

		highAngle, haTof := CalcAngleHigh(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected], airResistanceBool)
		lowAngle, laTof := CalcAngleLow(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected], airResistanceBool)

		switch airResistanceBool {
		case false:
			lowAngleGraph := GraphShotNoDrag(dist, lowAngle, curGun[chargeSelection.Selected], float64(gunAlt), float64(targetAlt), "Low Angle")
			highAngleGraph := GraphShotNoDrag(dist, highAngle, curGun[chargeSelection.Selected], float64(gunAlt), float64(targetAlt), "High Angle")

			highAngleImage.Image = highAngleGraph
			lowAngleImage.Image = lowAngleGraph

			highAngleImage.FillMode = canvas.ImageFillOriginal
			lowAngleImage.FillMode = canvas.ImageFillOriginal

			highAngleImage.Refresh()
			lowAngleImage.Refresh()
		case true:
			lowAngleGraph := GraphShotDrag(lastVectorPlotLow, dist, "Low Angle")
			highAngleGraph := GraphShotDrag(lastVectorPlotHigh, dist, "High Angle")

			highAngleImage.Image = highAngleGraph
			lowAngleImage.Image = lowAngleGraph

			highAngleImage.FillMode = canvas.ImageFillOriginal
			lowAngleImage.FillMode = canvas.ImageFillOriginal

			highAngleImage.Refresh()
			lowAngleImage.Refresh()
		}

		az, err := CalcAzimuth(gunGrid.Text, targetGrid.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		azimuthBoxGrid.Text = fmt.Sprintf("Gun Azimuth %f", az)
		azimuthBoxGrid.Refresh()
		gunElevationGrid.Text = fmt.Sprintf("High Angle: %d | Low Angle: %d", int(highAngle), int(lowAngle))
		gunElevationGrid.Refresh()
		lastCalcMission = FireMission{}
		lastCalcMission.Type = 1
		lastCalcMission.TargetGrid = targetGrid.Text
		lastCalcMission.TargetAlt, err = strconv.Atoi(targetAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		timeOfFlight.SetText(fmt.Sprintf("HA: %f | LA: %f", haTof, laTof))
		go Solution()
	})

	haTofButton := widget.NewButton("HA ToF Alarm", func() {
		go func() {
			splitTof := strings.Split(timeOfFlight.Text, "|")
			ha := strings.Split(strings.ReplaceAll(splitTof[0], " ", ""), ":")[1]
			haInt, err := strconv.Atoi(ha)
			if err != nil {
				a.SendNotification(fyne.NewNotification("Error", "Check if ToF is populated"))
				return
			}

			if haInt > 5 {
				haInt = haInt - 5
			} else {
				if !mute {
					Splash()
				}

				return
			}

			time.Sleep(time.Duration(haInt) * time.Second)
			if !mute {
				Splash()
			}
		}()
	})

	laTofButton := widget.NewButton("LA ToF Alarm", func() {
		go func() {
			splitTof := strings.Split(timeOfFlight.Text, "|")
			la := strings.Split(strings.ReplaceAll(splitTof[1], " ", ""), ":")[1]
			laInt, err := strconv.Atoi(la)
			if err != nil {
				a.SendNotification(fyne.NewNotification("Error", "Check if ToF is populated"))
				return
			}
			if laInt > 5 {
				laInt = laInt - 5
			} else {
				if !mute {
					Splash()
				}
				return
			}

			time.Sleep(time.Duration(laInt) * time.Second)
			if !mute {
				Splash()
			}
		}()
	})

	polarGridLabel := widget.NewLabel("")

	calculateMissionPolar := widget.NewButton("Calculate Mission", func() {
		dist, err := strconv.Atoi(distancePolar.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		gunAlt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		targetAlt, err := strconv.Atoi(targetAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}

		if curGun == nil {
			a.SendNotification(fyne.NewNotification("Error", "No gun selected"))
			return
		}

		if chargeSelection.Selected == "" {
			a.SendNotification(fyne.NewNotification("Error", "No charge selected"))
			return
		}

		highAngle, haTof := CalcAngleHigh(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected], airResistanceBool)
		lowAngle, laTof := CalcAngleLow(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected], airResistanceBool)

		switch airResistanceBool {
		case false:
			lowAngleGraph := GraphShotNoDrag(float64(dist), lowAngle, curGun[chargeSelection.Selected], float64(gunAlt), float64(targetAlt), "Low Angle")
			highAngleGraph := GraphShotNoDrag(float64(dist), highAngle, curGun[chargeSelection.Selected], float64(gunAlt), float64(targetAlt), "High Angle")

			highAngleImage.Image = highAngleGraph
			lowAngleImage.Image = lowAngleGraph

			highAngleImage.FillMode = canvas.ImageFillOriginal
			lowAngleImage.FillMode = canvas.ImageFillOriginal

			highAngleImage.Refresh()
			lowAngleImage.Refresh()
		case true:
			lowAngleGraph := GraphShotDrag(lastVectorPlotLow, float64(dist), "Low Angle")
			highAngleGraph := GraphShotDrag(lastVectorPlotHigh, float64(dist), "High Angle")

			highAngleImage.Image = highAngleGraph
			lowAngleImage.Image = lowAngleGraph

			highAngleImage.FillMode = canvas.ImageFillOriginal
			lowAngleImage.FillMode = canvas.ImageFillOriginal

			highAngleImage.Refresh()
			lowAngleImage.Refresh()
		}

		gunElevationPolar.Text = fmt.Sprintf("High Angle: %f | Low Angle: %f", highAngle, lowAngle)
		gunElevationPolar.Refresh()
		polarGrid, _ := PolarToGrid(distancePolar.Text, azimuthBoxPolar.Text, gunGrid.Text)
		polarGridLabel.SetText(polarGrid)
		polarGridLabel.Refresh()

		lastCalcMission = FireMission{}
		lastCalcMission.Type = 0
		lastCalcMission.TargetGrid = polarGrid
		lastCalcMission.TargetAlt, err = strconv.Atoi(targetAltitude.Text)

		timeOfFlight.SetText(fmt.Sprintf("HA: %f | LA: %f", haTof, laTof))
		if !mute {
			go Solution()
		}
	})

	otlAngle := widget.NewEntry()
	otlDirectionX := widget.NewSelect([]string{"Left", "Right"}, nil)
	otlDirectionX.SetSelected("Left")
	otlDirectionY := widget.NewSelect([]string{"Add", "Drop"}, nil)
	otlDirectionY.SetSelected("Add")
	otlDistanceY := widget.NewEntry()
	otlDistanceX := widget.NewEntry()

	otlAngle.PlaceHolder = "Observer to Target Line in MILS"
	otlDistanceY.PlaceHolder = "Adjust add/drop distance"
	otlDistanceX.PlaceHolder = "Adjust left/right distance"
	otlCalc := widget.NewLabel("")

	otlCalculate := widget.NewButton("Calculate OTL Adjust", func() {
		otlDistYInt, err := strconv.Atoi(otlDistanceY.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your Add/Drop distance"))
			return
		}
		
		otlDistXInt, err := strconv.Atoi(otlDistanceX.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your Left/Right distance"))
			return
		}

		otlAngleInt, err := strconv.Atoi(otlAngle.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your Observer to target Angle"))
			return
		}

		gunAltInt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your Gun altitude"))
			return
		}

		distance, _ := CalcDistance(gunGrid.Text, lastCalcMission.TargetGrid)
		origAz, _ := CalcAzimuth(gunGrid.Text, lastCalcMission.TargetGrid)
		otlCalc.SetText(OTLAdjust(float64(gunAltInt), float64(lastCalcMission.TargetAlt), distance, float64(otlDistYInt), float64(otlDistXInt), 
			float64(otlAngleInt), origAz, curGun[chargeSelection.Selected], otlDirectionY.Selected, otlDirectionX.Selected, airResistanceBool))
		if !mute {
			go Solution()
		}
		otlCalc.Refresh()
	})

	boxWidth := widget.NewEntry()
	boxSpread := widget.NewEntry()
	boxHeight := widget.NewEntry()
	boxOrientation := widget.NewEntry()

	boxWidth.PlaceHolder = "Width of Box"
	boxHeight.PlaceHolder = "Height of Box"
	boxSpread.PlaceHolder = "Distance between Shots"
	boxOrientation.PlaceHolder = "Orientation of Box (Mils)"
	boxCalculate := widget.NewButton("Calculate Box Sheaf", func() {
		boxWidthInt, err := strconv.Atoi(boxWidth.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box size"))
			return
		}
		boxSpreadInt, err := strconv.Atoi(boxSpread.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box spread"))
			return
		}

		gunAltInt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		boxOrientationInt, err := strconv.Atoi(boxOrientation.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		boxHeightInt, err := strconv.Atoi(boxHeight.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		shotsGrids := DrawARect(lastCalcMission.TargetGrid, float64(boxWidthInt), float64(boxHeightInt), float64(boxSpreadInt), float64(boxOrientationInt))

		shots := []string{}
		for _, shot := range shotsGrids[1:] {
			boxAz, err := CalcAzimuth(gunGrid.Text, shot)
			if err != nil {
				a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
				return
			}
			boxDist, err := CalcDistance(gunGrid.Text, shot)
			if err != nil {
				a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
				return
			}

			highAngle, _ := CalcAngleHigh(float64(gunAltInt), float64(lastCalcMission.TargetAlt), boxDist, curGun[chargeSelection.Selected], airResistanceBool)
			lowAngle, _ := CalcAngleLow(float64(gunAltInt), float64(lastCalcMission.TargetAlt), boxDist, curGun[chargeSelection.Selected], airResistanceBool)

			shots = append(shots, fmt.Sprintf("%s - AZ: %d | HA: %d | LA: %d", shot, int(boxAz), int(highAngle), int(lowAngle)))
		}

		shotWindow := a.NewWindow("Box Sheaf")
		closeButton := widget.NewButton("Close", func() {
			shotWindow.Close()
		})
		shotWindow.SetContent(container.NewBorder(nil, closeButton, nil, nil,
			widget.NewList(func() int {
				return len(shots)
			}, func() fyne.CanvasObject {
				return widget.NewLabel("")
			}, func(id widget.ListItemID, object fyne.CanvasObject) {
				object.(*widget.Label).SetText(strconv.Itoa(id) + ": " + shots[id])
			})))
		shotWindow.Resize(fyne.Size{
			Width:  500,
			Height: 400,
		})
		go Solution()
		shotWindow.Show()
	})

	linearCount := widget.NewEntry()
	linearDispersion := widget.NewEntry()
	linearDirection := widget.NewEntry()
	linearCalculate := widget.NewButton("Calculate Linear Sheaf", func() {
		shots := []string{}
		shots = append(shots, "First shot is the original calculated point")
		numShots, err := strconv.Atoi(linearCount.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your number of rounds"))
			return
		}

		dispersion, err := strconv.Atoi(linearDispersion.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		linearAz, err := strconv.Atoi(linearDirection.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion direction"))
			return
		}

		gunAltInt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		distance, _ := CalcDistance(gunGrid.Text, lastCalcMission.TargetGrid)
		origAz, _ := CalcAzimuth(gunGrid.Text, lastCalcMission.TargetGrid)
		for i := 0; i <= numShots; i++ {
			nextTarget := LinearSheaf(float64(gunAltInt), float64(lastCalcMission.TargetAlt), distance, float64(dispersion*i), float64(linearAz), origAz, curGun[chargeSelection.Selected], airResistanceBool)
			shots = append(shots, nextTarget)
		}

		shotWindow := a.NewWindow("Linear Adjust/Sheaf")
		closeButton := widget.NewButton("Close", func() {
			shotWindow.Close()
		})

		shotWindow.SetContent(container.NewBorder(nil, closeButton, nil, nil,
			widget.NewList(func() int {
				return len(shots)
			}, func() fyne.CanvasObject {
				return widget.NewLabel("")
			}, func(id widget.ListItemID, object fyne.CanvasObject) {
				object.(*widget.Label).SetText(strconv.Itoa(id) + ": " + shots[id])
			})))
		shotWindow.Resize(fyne.Size{
			Width:  500,
			Height: 400,
		})
		go Solution()
		shotWindow.Show()

	})
	linearCount.PlaceHolder = "Number of shots"
	linearDispersion.PlaceHolder = "Dispersion of rounds"
	linearDirection.PlaceHolder = "Direction of Shift"

	adjustMissions := container.NewTabItem("Adjustments", container.NewVScroll(container.NewVBox(
		widget.NewLabel("Observer to Target Adjusts"),
		otlAngle,		
		otlDirectionY,
		otlDistanceY,
		otlDirectionX,
		otlDistanceX,
		otlCalc,
		otlCalculate,
		widget.NewSeparator(),
		widget.NewLabel("Box Sheaf"),
		boxWidth,
		boxHeight,
		boxSpread,
		boxOrientation,
		boxCalculate,
		widget.NewSeparator(),
		widget.NewLabel("Linear Sheafs"),
		linearDirection,
		linearDispersion,
		linearCount,
		linearCalculate,
	)))

	gridMission := container.NewTabItem("Grid Missions", container.NewVScroll(container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Altitude"), gunAltitude, widget.NewSeparator(),
		widget.NewLabel("Target Grid"), targetGrid, widget.NewSeparator(),
		widget.NewLabel("Target Altitude"), targetAltitude, widget.NewSeparator(),
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Elevation"), gunElevationGrid, widget.NewSeparator(),
		widget.NewLabel("Time Of Flight"), timeOfFlight, widget.NewSeparator(),
		container.NewGridWithColumns(2, container.NewHBox(calculateMissionGrid, saveMissionButtonGrid, airResistance), targetName),
		container.NewHScroll(container.NewHBox(highAngleImage, lowAngleImage)),
	)))

	polarMission := container.NewTabItem("Polar Mission", container.NewVScroll(container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Altitude"), gunAltitude, widget.NewSeparator(),
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxPolar, widget.NewSeparator(),
		widget.NewLabel("Target Altitude"), targetAltitude, widget.NewSeparator(),
		widget.NewLabel("Distance to Target"), distancePolar, widget.NewSeparator(),
		widget.NewLabel("Gun Elevation"), gunElevationPolar, widget.NewSeparator(),
		widget.NewLabel("Time Of Flight"), timeOfFlight, widget.NewSeparator(),
		widget.NewLabel("Target Calculated Grid"), polarGridLabel, widget.NewSeparator(),
		container.NewGridWithColumns(2, container.NewHBox(calculateMissionPolar, saveMissionButtonGrid, airResistance), targetName),
		container.NewHScroll(container.NewHBox(highAngleImage, lowAngleImage)),
	)))

	savedMissionList := widget.NewList(
		func() int {
			return len(savedMissions)
		}, func() fyne.CanvasObject {
			return widget.NewLabel("")
		}, func(id widget.ListItemID, object fyne.CanvasObject) {
			object.(*widget.Label).SetText(fmt.Sprintf("%d", id) + ": " + savedMissions[id].Name)
		})

	deleteMission := widget.NewButton("Delete Mission", func() {
		if len(savedMissions) == 0 {
			return
		}
		popup := dialog.NewEntryDialog("Delete Mission", "Enter Mission Number", func(s string) {
			missionInt, err := strconv.Atoi(s)
			if err != nil {
				return
			}
			savedMissions = savedMissions.Remove(missionInt)
			savedMissionList.Refresh()
		}, w)
		popup.Show()
	})

	savedMissionsTab := container.NewTabItem("Saved Missions", container.NewBorder(nil, deleteMission, nil, nil, savedMissionList))

	pressure := widget.NewEntry()
	pressure.SetPlaceHolder("Air Pressure (hPA)")
	temperature := widget.NewEntry()
	temperature.SetPlaceHolder("Temperature (C)")
	humidity := widget.NewEntry()
	humidity.SetPlaceHolder("Humidity (%)")
	airDensityBox := widget.NewLabel("")
	airDensityCoefBox := widget.NewLabel("")
	calcDensity := widget.NewButton("Calculate Air Density", func() {
		pressureInt, _ := strconv.ParseFloat(pressure.Text, 64)
		temperatureInt, _ := strconv.ParseFloat(temperature.Text, 64)
		humidityInt, _ := strconv.ParseFloat(humidity.Text, 64)
		humidityFloat := float64(humidityInt) / 100
		pressureFloat := float64(pressureInt) * 100
		sat := 610.78 * math.Pow(10, (7.5*temperatureInt)/(temperatureInt+237.3))
		vap := humidityFloat * sat
		partialPressure := pressureFloat - vap
		airDensity := (partialPressure*0.028964 + vap*0.018016) / (8.314 * (temperatureInt + 273.15))
		airDensityCoef := ((airDensity / 1.225) - 1) * 100
		airDensityBox.SetText(fmt.Sprintf("%f kg/m³", airDensity))
		airDensityBox.Refresh()
		airDensityCoefBox.SetText(fmt.Sprintf("%f pct", airDensityCoef))
		airDensityCoefBox.Refresh()
	})

	degreesToMilsCalcLabel := widget.NewLabel("")
	degreesToMilsCalcEntry := widget.NewEntry()
	degreesToMilsCalcEntry.SetPlaceHolder("Degrees")
	degreesToMilsCalcButton := widget.NewButton("Calculate", func() {
		degreesInt, err := strconv.Atoi(degreesToMilsCalcEntry.Text)
		if err != nil {
			return
		}
		mils := float64(degreesInt) * 17.777778

		degreesToMilsCalcLabel.SetText(fmt.Sprintf("%f", mils))
		degreesToMilsCalcLabel.Refresh()

	})

	degreesMilsImage := canvas.NewImageFromResource(fyne.NewStaticResource("degreesToMils", degreeMils))
	degreesMilsImage.FillMode = canvas.ImageFillOriginal

	observerPos := widget.NewEntry()
	observerPos.SetPlaceHolder("Observer Position")
	targetAz := widget.NewEntry()
	targetAz.SetPlaceHolder("Target Azimuth")
	targetDist := widget.NewEntry()
	targetDist.SetPlaceHolder("Target Distance")
	targetPos := widget.NewEntry()
	targetPos.SetPlaceHolder("Target Position")

	calculatePolar := widget.NewButton("Calculate Polar From Observer", func() {
		targetPosCalc, err := PolarToGrid(targetDist.Text, targetAz.Text, observerPos.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		targetPos.SetText(targetPosCalc)
		targetPos.Refresh()
	})

	habTacUrl := widget.NewEntry()
	habTacUrl.SetPlaceHolder("HabTac URL")
	habTacSession := widget.NewEntry()
	habTacSession.SetPlaceHolder("HabTac Session")
	habTacGunName := widget.NewEntry()
	habTacGunName.SetPlaceHolder("HabTac Gun Name")

	go func() {
		for {
			if habTacSync {
				habTacGunName.Disable()
				gunGrid.Disable()
				if habTacUrl.Text == "" || habTacSession.Text == "" {
					habTacLogin.SetChecked(false)
					continue
				}
				_, err2 := url.Parse(habTacUrl.Text)
				if err2 != nil {
					habTacLogin.SetChecked(false)
					continue
				}
				if gunGrid.Text != "" {
					_, err := http.Get(habTacUrl.Text + deleteMarker + "?session=" + habTacSession.Text + "&title=" + habTacGunName.Text)
					if err != nil {
						a.SendNotification(fyne.NewNotification("Error", err.Error()))
						continue
					}
					northing := ""
					easting := ""
					switch len(gunGrid.Text) {
					case 6:
						northing = gunGrid.Text[:3] + "00"
						easting = gunGrid.Text[3:] + "00"
					case 8:
						northing = gunGrid.Text[:4] + "0"
						easting = gunGrid.Text[4:] + "0"
					case 10:
						northing = gunGrid.Text[:5]
						easting = gunGrid.Text[5:]

					}
					reqForm := url.Values{
						"lat":      {easting},
						"lng":      {northing},
						"elv":      {gunAltitude.Text},
						"title":    {habTacGunName.Text},
						"unit":     {"savage"},
						"comments": {gunSelection.Selected},
						"session":  {habTacSession.Text},
						"id":       {sessionUuid},
					}

					_, err = http.PostForm(habTacUrl.Text+addMarker, reqForm)
					if err != nil {
						a.SendNotification(fyne.NewNotification("Error", err.Error()))
						continue
					}

				}

				totalUrl := habTacUrl.Text + "/api/markers?session=" + habTacSession.Text
				resp, err := http.Get(totalUrl)
				if err != nil {
					a.SendNotification(fyne.NewNotification("Error", err.Error()))
					continue
				}
				markers := []Marker{}
				err = json.NewDecoder(resp.Body).Decode(&markers)
				if err != nil {
					a.SendNotification(fyne.NewNotification("Error", err.Error()))
					continue
				}
				savedMissions = []FireMission{}
				for _, marker := range markers {
					if marker.Unit != "fm" {
						continue
					}
					elvConversion, err := strconv.Atoi(marker.Elv)
					if err != nil {
						continue
					}
					savedMissions = append(savedMissions, FireMission{
						Name:       marker.Title,
						TargetGrid: marker.Lng + marker.Lat,
						TargetAlt:  elvConversion,
						Type:       1,
					})
				}
			} else {
				habTacGunName.Enable()
				gunGrid.Enable()
			}
			savedMissionList.Refresh()
			time.Sleep(1 * time.Second)
		}
	}()

	utilityTab := container.NewTabItem("Utilities", container.NewVScroll(container.NewVBox(widget.NewLabel("Polar from Observer Grid Calculator"),
		observerPos, targetAz, targetDist, calculatePolar, targetPos, widget.NewLabel("Air Density Calculator"),
		pressure, temperature, humidity, airDensityBox, airDensityCoefBox, calcDensity, widget.NewSeparator(),
		degreesMilsImage, widget.NewSeparator(), widget.NewLabel("Degrees to Mils Conversions"), degreesToMilsCalcEntry,
		degreesToMilsCalcLabel, degreesToMilsCalcButton, widget.NewSeparator(), widget.NewLabel("HabTac Sync"),
		habTacUrl, habTacSession, habTacGunName, habTacLogin, widget.NewSeparator(), widget.NewLabel("Mute Audio"), muteButton)))

	infoTab := container.NewTabItemWithIcon("Info", theme.ListIcon(), container.NewVBox(
		widget.NewLabel("Thanks to the following people:"),
		widget.NewLabel("Gens - For his original spreadsheet"),
		widget.NewLabel("Cashton - For the audio lines"),
		muteButton,
	))

	missionTabs := container.NewAppTabs(gridMission, polarMission, adjustMissions, savedMissionsTab, utilityTab, infoTab)

	savedMissionList.OnSelected = func(id widget.ListItemID) {

		missionTabs.SelectIndex(0)
		targetGrid.SetText(savedMissions[id].TargetGrid)
		targetAltitude.SetText(strconv.Itoa(savedMissions[id].TargetAlt))
		targetAltitude.Refresh()
		targetGrid.Refresh()

		gunAlt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		distanceRecalc, err := CalcDistance(gunGrid.Text, savedMissions[id].TargetGrid)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		highAngle, _ := CalcAngleHigh(float64(gunAlt), float64(savedMissions[id].TargetAlt), distanceRecalc, curGun[chargeSelection.Selected], airResistanceBool)
		lowAngle, _ := CalcAngleLow(float64(gunAlt), float64(savedMissions[id].TargetAlt), distanceRecalc, curGun[chargeSelection.Selected], airResistanceBool)
		az, err := CalcAzimuth(gunGrid.Text, targetGrid.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		azimuthBoxGrid.Text = fmt.Sprintf("Gun Azimuth %f", az)
		azimuthBoxGrid.Refresh()
		gunElevationGrid.Text = fmt.Sprintf("High Angle: %d | Low Angle: %d", int(highAngle), int(lowAngle))
		gunElevationGrid.Refresh()
		lastCalcMission = FireMission{}
		lastCalcMission.Type = 1
		lastCalcMission.TargetGrid = targetGrid.Text
		lastCalcMission.TargetAlt, err = strconv.Atoi(targetAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
		go Solution()
		savedMissionList.UnselectAll()
	}

	gunSelectRow.Add(haTofButton)
	gunSelectRow.Add(laTofButton)

	w.SetContent(container.NewBorder(gunSelectRow, nil, nil, nil, missionTabs))
	w.ShowAndRun()
}
