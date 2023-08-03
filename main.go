package main

import (
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
)

//go:embed artillery.json
var artilleryJson []byte

var g float64 = 9.80665

var r2m float64 = 1018.591636

func CalcAngleHigh(initHeight, finalHeight, distance, velocity float64) float64 {
	delta := finalHeight - initHeight
	tanThetaHigh := (float64(velocity*velocity) + math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)
	return math.Atan(tanThetaHigh) * r2m
}

func CalcAngleLow(initHeight, finalHeight, distance, velocity float64) float64 {
	delta := finalHeight - initHeight
	tanThetaLow := (float64(velocity*velocity) - math.Sqrt(velocity*velocity*velocity*velocity-(g*(g*(distance*distance)+(2*delta)*(velocity*velocity))))) / (g * distance)
	return math.Atan(tanThetaLow) * r2m
}

func TimeOfFlight(velocity, angle float64) string {
	tof := (2 * velocity * math.Sin(angle)) / g

	return strconv.Itoa(int(tof))
}

func LinearSheaf(gunHeight, targetHeight, origDistance, shiftDistance, shiftAzimuth, azimuth, velocity float64) string {
	xo := origDistance * math.Cos(azimuth/r2m)

	yo := origDistance * math.Sin(azimuth/r2m)

	xs := shiftDistance * math.Cos(shiftAzimuth/r2m)

	ys := shiftDistance * math.Sin(shiftAzimuth/r2m)

	xf := xo + xs
	yf := yo + ys

	newDistance := math.Sqrt((xf * xf) + (yf + yf))

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

	low := CalcAngleLow(gunHeight, targetHeight, newDistance, velocity)
	high := CalcAngleHigh(gunHeight, targetHeight, newDistance, velocity)

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
	northingDelta := math.Sin(float64(az)/r2m) * distanceFloat
	eastingDelta := math.Cos(float64(az)/r2m) * distanceFloat

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

	eastingGun := gunPos[:5]
	northingGun := gunPos[5:]

	eastingGunInt, err := strconv.Atoi(eastingGun)
	northingGunInt, err := strconv.Atoi(northingGun)

	targetNorthing := northingGunInt + int(northingDelta)
	targetEasting := eastingGunInt + int(eastingDelta)

	targetGrid := fmt.Sprintf("%d%d", targetEasting, targetNorthing)
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

	azMils := math.Atan(float64(deltaEasting)/float64(deltaNorthing)) * r2m
	if azMils < 0 {
		azMils = azMils + 6400
	}

	return azMils, nil

}

func main() {
	lastCalcMission := FireMission{}
	var savedMissions []FireMission

	guns := Guns{}

	curGun := Gun{}

	err := json.Unmarshal(artilleryJson, &guns)
	if err != nil {
		return
	}

	a := app.New()
	a.Settings().SetTheme(theme.DarkTheme())
	w := a.NewWindow("CFF Calculator")
	azimuthBoxGrid := widget.NewLabel("")
	gunElevationPolar := widget.NewLabel("")
	gunElevationGrid := widget.NewLabel("")

	timeOfFlight := widget.NewLabel("TOF: ")

	azimuthBoxPolar := widget.NewEntry()
	distancePolar := widget.NewEntry()

	gunGrid := widget.NewEntry()
	gunAltitude := widget.NewEntry()

	targetGrid := widget.NewEntry()
	targetAltitude := widget.NewEntry()

	saveMissionButtonGrid := widget.NewButton("SaveMission", func() {

		fmRecordWindow := a.NewWindow("Record Fire Mission")

		fmNameEntry := widget.NewEntry()

		closeFmWindow := widget.NewButton("Save", func() {
			lastCalcMission.Name = fmNameEntry.Text
			savedMissions = append(savedMissions, lastCalcMission)
			fmRecordWindow.Close()
		})

		fmRecordWindow.SetContent(container.NewVBox(widget.NewLabel("FM Name:"), fmNameEntry, closeFmWindow))

		fmRecordWindow.Resize(fyne.Size{
			Width:  400,
			Height: 300,
		})
		fmRecordWindow.Show()

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

		highAngle := CalcAngleHigh(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected])
		lowAngle := CalcAngleLow(float64(gunAlt), float64(targetAlt), dist, curGun[chargeSelection.Selected])
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
		haTof := TimeOfFlight(curGun[chargeSelection.Selected], highAngle/r2m)
		laTof := TimeOfFlight(curGun[chargeSelection.Selected], lowAngle/r2m)

		timeOfFlight.SetText(fmt.Sprintf("HA: %s | LA: %s", haTof, laTof))
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

		highAngle := CalcAngleHigh(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected])
		lowAngle := CalcAngleLow(float64(gunAlt), float64(targetAlt), float64(dist), curGun[chargeSelection.Selected])
		gunElevationPolar.Text = fmt.Sprintf("High Angle: %f | Low Angle: %f", highAngle, lowAngle)
		gunElevationPolar.Refresh()
		polarGrid, _ := PolarToGrid(distancePolar.Text, azimuthBoxPolar.Text, gunGrid.Text)
		polarGridLabel.SetText(polarGrid)
		polarGridLabel.Refresh()

		lastCalcMission = FireMission{}
		lastCalcMission.Type = 0
		lastCalcMission.TargetGrid = polarGrid
		lastCalcMission.TargetAlt, err = strconv.Atoi(targetAltitude.Text)

		haTof := TimeOfFlight(curGun[chargeSelection.Selected], highAngle/r2m)
		laTof := TimeOfFlight(curGun[chargeSelection.Selected], lowAngle/r2m)

		timeOfFlight.SetText(fmt.Sprintf("HA: %s | LA: %s", haTof, laTof))
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
		container.NewHBox(calculateMissionGrid, saveMissionButtonGrid)))

	polarMission := container.NewTabItem("Polar Mission", container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid, widget.NewSeparator(),
		widget.NewLabel("Gun Altitude"), gunAltitude, widget.NewSeparator(),
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxPolar, widget.NewSeparator(),
		widget.NewLabel("Target Altitude"), targetAltitude, widget.NewSeparator(),
		widget.NewLabel("Distance to Target"), distancePolar, widget.NewSeparator(),
		widget.NewLabel("Gun Elevation"), gunElevationPolar, widget.NewSeparator(),
		widget.NewLabel("Time Of Flight"), timeOfFlight, widget.NewSeparator(),
		widget.NewLabel("Target Calculated Grid"), polarGridLabel, widget.NewSeparator(),
		container.NewHBox(calculateMissionPolar, saveMissionButtonGrid)))

	savedMissionList := widget.NewList(
		func() int {
			return len(savedMissions)
		}, func() fyne.CanvasObject {
			return widget.NewLabel("")
		}, func(id widget.ListItemID, object fyne.CanvasObject) {
			object.(*widget.Label).SetText(savedMissions[id].Name)
		})

	savedMissionsTab := container.NewTabItem("Saved Missions", savedMissionList)

	missionTabs := container.NewAppTabs(gridMission, polarMission, adjustMissions, savedMissionsTab)

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
		highAngle := CalcAngleHigh(float64(gunAlt), float64(savedMissions[id].TargetAlt), distanceRecalc, curGun[chargeSelection.Selected])
		lowAngle := CalcAngleLow(float64(gunAlt), float64(savedMissions[id].TargetAlt), distanceRecalc, curGun[chargeSelection.Selected])
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
		savedMissionList.UnselectAll()
	}

	w.SetContent(container.NewBorder(gunSelectRow, nil, nil, nil, missionTabs))
	w.ShowAndRun()
}
