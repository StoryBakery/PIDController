--!native

local PIDController = require("./PIDController")

local TAU = math.pi * 2

export type AnglePIDControllerParams = {
	Kp: number?,
	Ki: number?,
	Kd: number?,
	Setpoint: number?,
	InitialMeasurement: number?,
	OutputMin: number?,
	OutputMax: number?,
	IntegralMin: number?,
	IntegralMax: number?,
	DerivativeMode: PIDController.DerivativeMode?,
	WrapPeriod: number?,
}

local AnglePIDController = {}
AnglePIDController.__index = AnglePIDController

export type AnglePIDController = setmetatable<{
	Setpoint: number,
	WrapPeriod: number,
	_controller: any,
	_lastRawMeasurement: number?,
	_continuousMeasurement: number?,
	_appliedSetpoint: number?,
}, typeof(AnglePIDController)>

local function wrapDelta(value: number, period: number): number
	local halfPeriod = period * 0.5
	return ((value + halfPeriod) % period) - halfPeriod
end

local function applySetpoint(self: AnglePIDController)
	local continuousSetpoint = self.Setpoint
	local continuousMeasurement = self._continuousMeasurement
	if continuousMeasurement then
		continuousSetpoint = continuousMeasurement
			+ wrapDelta(self.Setpoint - continuousMeasurement, self.WrapPeriod)
	end

	self._controller:SetSetpoint(continuousSetpoint)
	self._appliedSetpoint = self.Setpoint
end

function AnglePIDController.new(params: AnglePIDControllerParams?): AnglePIDController
	params = params or {}

	local wrapPeriod = params.WrapPeriod or TAU
	assert(wrapPeriod > 0, "WrapPeriod must be > 0")

	local controller = PIDController.new({
		Kp = params.Kp,
		Ki = params.Ki,
		Kd = params.Kd,
		Setpoint = params.Setpoint,
		InitialMeasurement = params.InitialMeasurement,
		OutputMin = params.OutputMin,
		OutputMax = params.OutputMax,
		IntegralMin = params.IntegralMin,
		IntegralMax = params.IntegralMax,
		DerivativeMode = params.DerivativeMode,
	})

	local self = setmetatable({
		Setpoint = controller.Setpoint,
		WrapPeriod = wrapPeriod,
		_controller = controller,
		_lastRawMeasurement = params.InitialMeasurement,
		_continuousMeasurement = params.InitialMeasurement,
		_appliedSetpoint = nil,
	}, AnglePIDController)

	applySetpoint(self)

	return self
end

function AnglePIDController:SetTunings(kp: number, ki: number, kd: number)
	self._controller:SetTunings(kp, ki, kd)
end

function AnglePIDController:SetKp(kp: number)
	self._controller:SetKp(kp)
end

function AnglePIDController:SetKi(ki: number)
	self._controller:SetKi(ki)
end

function AnglePIDController:SetKd(kd: number)
	self._controller:SetKd(kd)
end

function AnglePIDController:SetDerivativeMode(mode: PIDController.DerivativeMode)
	self._controller:SetDerivativeMode(mode)
end

function AnglePIDController:SetSetpoint(setpoint: number)
	self.Setpoint = setpoint
	applySetpoint(self)
end

function AnglePIDController:SetOutputLimits(minValue: number, maxValue: number)
	self._controller:SetOutputLimits(minValue, maxValue)
end

function AnglePIDController:SetIntegralLimits(minValue: number, maxValue: number)
	self._controller:SetIntegralLimits(minValue, maxValue)
end

function AnglePIDController:ClearOutputLimits()
	self._controller:ClearOutputLimits()
end

function AnglePIDController:ClearIntegralLimits()
	self._controller:ClearIntegralLimits()
end

function AnglePIDController:Reset(measurement: number?)
	self._controller:Reset(measurement)
	self._lastRawMeasurement = measurement
	self._continuousMeasurement = measurement
	self._appliedSetpoint = nil
end

function AnglePIDController:GetMeasurement(): number?
	return self._lastRawMeasurement
end

function AnglePIDController:GetContinuousMeasurement(): number?
	return self._continuousMeasurement
end

function AnglePIDController:Update(dt: number, measurement: number): number
	local continuousMeasurement = measurement
	local lastRawMeasurement = self._lastRawMeasurement
	if lastRawMeasurement then
		local previousContinuous = self._continuousMeasurement or lastRawMeasurement
		continuousMeasurement = previousContinuous
			+ wrapDelta(measurement - lastRawMeasurement, self.WrapPeriod)
	end

	self._lastRawMeasurement = measurement
	self._continuousMeasurement = continuousMeasurement

	if self._appliedSetpoint ~= self.Setpoint then
		applySetpoint(self)
	end

	return self._controller:Update(dt, continuousMeasurement)
end

return AnglePIDController
