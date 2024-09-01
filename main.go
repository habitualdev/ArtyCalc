package main

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/dialog"
	"fyne.io/fyne/v2/theme"
	"fyne.io/fyne/v2/widget"
	"github.com/google/uuid"
	"gonum.org/v1/plot/plotter"
	"image"
	_ "image/png"
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
