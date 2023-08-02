package main

var polarMissionType = 0
var gridMissionType = 1

type Gun map[string]float64

type Guns map[string]Gun

type FireMission struct {
	Type       int
	Name       string
	TargetGrid string
	TargetAlt  int
	Azimuth    float64
	Distance   float64
}
