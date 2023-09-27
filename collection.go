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
}

type FireMissions []FireMission

func (f FireMissions) Remove(i int) FireMissions {
	if i >= len(f) {
		return f
	}
	return append(f[:i], f[i+1:]...)
}
