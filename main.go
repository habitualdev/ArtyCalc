package main

import (
	"C"
	_ "embed"
	"encoding/json"
	"errors"
	"fmt"
	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/theme"
	"fyne.io/fyne/v2/widget"
	"math"
	"strconv"
	"strings"
	"time"
)

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

var a fyne.App

var mute = false

var testAngle = 0.0

var muz = 0.0

func CalcAngleHigh(initHeight, finalHeight, distance, velocity float64, drag bool) (float64, float64) {
	delta := finalHeight - initHeight
	tanThetaHigh := (float64(velocity*velocity) + math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)

	angle := math.Atan(tanThetaHigh) * r2m

	if !drag {
		return angle, TimeOfFlight(velocity, angle)
	}

	muz = velocity
	for testAngle = angle; testAngle != 0; testAngle-- {
		dist, tof := CalculateForAngle(delta, testAngle, muz)
		if math.Abs(distance-dist) < 2 {

			return testAngle, tof
		}
	}
	return 0, 0
}

func CalcAngleLow(initHeight, finalHeight, distance, velocity float64, drag bool) (float64, float64) {
	delta := finalHeight - initHeight
	tanThetaLow := (float64(velocity*velocity) - math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)
	angle := math.Atan(tanThetaLow) * r2m
	if !drag {
		return angle, TimeOfFlight(velocity, angle)
	}
	muz = velocity
	for testAngle = angle; testAngle != 1600; testAngle++ {
		dist, tof := CalculateForAngle(delta, testAngle, muz)
		if math.Abs(distance-dist) < 2 {

			return testAngle, tof
		}
	}
	return 0, 0
}

func TimeOfFlight(velocity, angle float64) float64 {
	tof := (2 * velocity * math.Sin(angle)) / g

	return tof
}

func LinearSheaf(gunHeight, targetHeight, origDistance, shiftDistance, shiftAzimuth, azimuth, velocity float64) string {
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

	low, _ := CalcAngleLow(gunHeight, targetHeight, newDistance, velocity, true)
	high, _ := CalcAngleHigh(gunHeight, targetHeight, newDistance, velocity, true)

	return fmt.Sprintf("Azimuth: %d | HA: %d | LA: %d", newAz, int(high), int(low))
}

func OTLAdjust(gunHeight, targetHeight, origDistance, shiftDistance, otlAzimuth, azimuth, velocity float64, direction string) string {
	adjustAzimuth := otlAzimuth
	switch direction {
	case "Left":
		adjustAzimuth = adjustAzimuth - 1600
		if adjustAzimuth < 0 {
			adjustAzimuth = adjustAzimuth + 6400
		}
	case "Right":
		adjustAzimuth = adjustAzimuth + 1600
		if adjustAzimuth > 0 {
			adjustAzimuth = adjustAzimuth - 6400
		}
	case "Subtract":
		adjustAzimuth = adjustAzimuth - 3200
		if adjustAzimuth < 0 {
			adjustAzimuth = adjustAzimuth + 6400
		}
	default:
		break
	}

	return LinearSheaf(gunHeight, targetHeight, origDistance, shiftDistance, adjustAzimuth, azimuth, velocity)
}

func BoxSheaf(width, gunHeight, targetHeight, origDistance, shiftDistance, azimuth, velocity float64) []string {
	shots := []string{"Impacts work from Top Left (original point) to Bottom Right"}
	shotsPerSide := width / shiftDistance

	rightAz := azimuth + 1600
	if rightAz > 6400 {
		rightAz = rightAz - 6400
	}

	for i := 0.0; i <= shotsPerSide; i++ {
		for j := 0.0; j <= shotsPerSide; j++ {
			shot := LinearSheaf(gunHeight, targetHeight, origDistance-(i*shiftDistance), j*shiftDistance, rightAz, azimuth, velocity)
			shots = append(shots, shot)
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

func main() {
	lastCalcMission := FireMission{}
	var savedMissions []FireMission
	var err error
	airResistanceBool := false
	guns := Guns{}

	curGun := Gun{}

	err = json.Unmarshal(artilleryJson, &guns)
	if err != nil {
		return
	}

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
		curGunCharges = []string{}
		for charge, _ := range curGun {
			curGunCharges = append(curGunCharges, charge)
		}
		chargeSelection.Options = curGunCharges
	})

	gunSelectRow := container.NewHBox(gunSelection, chargeSelection)

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

		highAngle, haTof := CalcAngleHigh(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected], airResistanceBool)
		lowAngle, laTof := CalcAngleLow(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected], airResistanceBool)
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

		highAngle, haTof := CalcAngleHigh(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected], airResistanceBool)
		lowAngle, laTof := CalcAngleLow(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected], airResistanceBool)
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
	otlDirection := widget.NewSelect([]string{"Add", "Subtract", "Left", "Right"}, nil)
	otlDirection.SetSelected("Add")
	otlDistance := widget.NewEntry()
	otlAngle.PlaceHolder = "Observer Target Angle"
	otlDistance.PlaceHolder = "Adjust Distance"
	otlCalc := widget.NewLabel("")

	otlCalculate := widget.NewButton("Calculate OTL Adjust", func() {
		otlDistInt, err := strconv.Atoi(otlDistance.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box size"))
			return
		}

		otlAngleInt, err := strconv.Atoi(otlAngle.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box size"))
			return
		}

		gunAltInt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		distance, _ := CalcDistance(gunGrid.Text, lastCalcMission.TargetGrid)
		origAz, _ := CalcAzimuth(gunGrid.Text, lastCalcMission.TargetGrid)
		otlCalc.SetText(OTLAdjust(float64(gunAltInt), float64(lastCalcMission.TargetAlt), distance, float64(otlDistInt),
			float64(otlAngleInt), origAz, curGun[chargeSelection.Selected], otlDistance.SelectedText()))
		if !mute {
			go Solution()
		}
		otlCalc.Refresh()
	})

	boxSize := widget.NewEntry()
	boxSpread := widget.NewEntry()
	boxSize.PlaceHolder = "Size of Box Sheaf"
	boxSpread.PlaceHolder = "Distance between Shots"
	boxCalculate := widget.NewButton("Calculate Box Sheaf", func() {
		boxSizeInt, err := strconv.Atoi(boxSize.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box size"))
			return
		}
		boxSpread, err := strconv.Atoi(boxSpread.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your box spread"))
			return
		}

		gunAltInt, err := strconv.Atoi(gunAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", "Check your dispersion distance"))
			return
		}

		distance, _ := CalcDistance(gunGrid.Text, lastCalcMission.TargetGrid)
		origAz, _ := CalcAzimuth(gunGrid.Text, lastCalcMission.TargetGrid)

		shots := BoxSheaf(float64(boxSizeInt), float64(gunAltInt), float64(lastCalcMission.TargetAlt), distance, float64(boxSpread), origAz, curGun[chargeSelection.Selected])
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
				object.(*widget.Label).SetText(shots[id])
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
			nextTarget := LinearSheaf(float64(gunAltInt), float64(lastCalcMission.TargetAlt), distance, float64(dispersion*i), float64(linearAz), origAz, curGun[chargeSelection.Selected])
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
				object.(*widget.Label).SetText(shots[id])
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
	adjustMissions := container.NewTabItem("Adjustments", container.NewVBox(
		widget.NewLabel("Observer to Target Adjusts"),
		otlDirection,
		otlAngle,
		otlDistance,
		otlCalc,
		otlCalculate,
		widget.NewSeparator(),
		widget.NewLabel("Box Sheaf"),
		boxSize,
		boxSpread,
		boxCalculate,
		widget.NewSeparator(),
		widget.NewLabel("Linear Sheafs"),
		linearDirection,
		linearDispersion,
		linearCount,
		linearCalculate,
	))

	gridMission := container.NewTabItem("Grid Missions", container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Altitude"), gunAltitude, widget.NewSeparator(),
		widget.NewLabel("Target Grid"), targetGrid, widget.NewSeparator(),
		widget.NewLabel("Target Altitude"), targetAltitude, widget.NewSeparator(),
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Elevation"), gunElevationGrid, widget.NewSeparator(),
		widget.NewLabel("Time Of Flight"), timeOfFlight, widget.NewSeparator(),
		container.NewGridWithColumns(2, container.NewHBox(calculateMissionGrid, saveMissionButtonGrid, airResistance), targetName),
	))

	polarMission := container.NewTabItem("Polar Mission", container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Altitude"), gunAltitude, widget.NewSeparator(),
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxPolar, widget.NewSeparator(),
		widget.NewLabel("Target Altitude"), targetAltitude, widget.NewSeparator(),
		widget.NewLabel("Distance to Target"), distancePolar, widget.NewSeparator(),
		widget.NewLabel("Gun Elevation"), gunElevationPolar, widget.NewSeparator(),
		widget.NewLabel("Time Of Flight"), timeOfFlight, widget.NewSeparator(),
		widget.NewLabel("Target Calculated Grid"), polarGridLabel, widget.NewSeparator(),
		container.NewGridWithColumns(2, container.NewHBox(calculateMissionPolar, saveMissionButtonGrid, airResistance), targetName),
	))

	savedMissionList := widget.NewList(
		func() int {
			return len(savedMissions)
		}, func() fyne.CanvasObject {
			return widget.NewLabel("")
		}, func(id widget.ListItemID, object fyne.CanvasObject) {
			object.(*widget.Label).SetText(savedMissions[id].Name)
		})

	savedMissionsTab := container.NewTabItem("Saved Missions", savedMissionList)

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

	utilityTab := container.NewTabItem("Utilities", container.NewVBox(pressure, temperature, humidity, airDensityBox, airDensityCoefBox, calcDensity))

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
