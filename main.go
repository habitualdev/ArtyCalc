package main

import (
	_ "embed"
	"encoding/json"
	"errors"
	"fmt"
	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
	"github.com/ncruces/zenity"
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
	w := a.NewWindow("CFF Calculator")
	azimuthBoxGrid := widget.NewLabel("")
	gunElevationPolar := widget.NewLabel("")
	gunElevationGrid := widget.NewLabel("")

	azimuthBoxPolar := widget.NewEntry()
	distancePolar := widget.NewEntry()

	gunGrid := widget.NewEntry()
	gunAltitude := widget.NewEntry()

	targetGrid := widget.NewEntry()
	targetAltitude := widget.NewEntry()

	saveMissionButtonGrid := widget.NewButton("SaveMission", func() {
		lastCalcMission.Name, err = zenity.Entry("Save Fire Mission as...")
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
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
		lastCalcMission.Distance = dist
		lastCalcMission.TargetAlt, err = strconv.Atoi(targetAltitude.Text)
		if err != nil {
			a.SendNotification(fyne.NewNotification("Error", err.Error()))
			return
		}
	})

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
	})

	gridMission := container.NewTabItem("Grid Missions", container.NewVBox(
		widget.NewLabel("Gun Position"), gunGrid,
		widget.NewLabel("Gun Altitude"), gunAltitude,
		widget.NewLabel("Target Grid"), targetGrid,
		widget.NewLabel("Target Altitude"), targetAltitude,
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxGrid,
		widget.NewLabel("Gun Elevation"), gunElevationGrid,
		container.NewHBox(saveMissionButtonGrid, calculateMissionGrid)))

	polarMission := container.NewTabItem("Polar Mission", container.NewVBox(
		widget.NewLabel("Gun Altitude"), gunAltitude,
		widget.NewLabel("Gun to Target Azimuth"), azimuthBoxPolar,
		widget.NewLabel("Target Altitude"), targetAltitude,
		widget.NewLabel("Distance to Target"), distancePolar,
		widget.NewLabel("Gun Elevation"), gunElevationPolar,
		container.NewHBox(calculateMissionPolar)))

	savedMissionList := widget.NewList(
		func() int {
			return len(savedMissions)
		}, func() fyne.CanvasObject {
			return widget.NewLabel("")
		}, func(id widget.ListItemID, object fyne.CanvasObject) {
			object.(*widget.Label).SetText(savedMissions[id].Name)
		})

	savedMissionsTab := container.NewTabItem("Saved Missions", savedMissionList)

	missionTabs := container.NewAppTabs(gridMission, polarMission, savedMissionsTab)

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

		highAngle := CalcAngleHigh(float64(gunAlt), float64(savedMissions[id].TargetAlt), savedMissions[id].Distance, curGun[chargeSelection.Selected])
		lowAngle := CalcAngleLow(float64(gunAlt), float64(savedMissions[id].TargetAlt), savedMissions[id].Distance, curGun[chargeSelection.Selected])
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
		lastCalcMission.Distance = savedMissions[id].Distance
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
