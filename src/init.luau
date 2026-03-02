--!native

local DEFAULT_MIN = -math.huge
local DEFAULT_MAX = math.huge

export type PIDControllerParams = {
	Kp: number?,
	Ki: number?,
	Kd: number?,
	Setpoint: number?,
	InitialMeasurement: number?,
	OutputMin: number?,
	OutputMax: number?,
	IntegralMin: number?,
	IntegralMax: number?,
	DerivativeOnMeasurement: boolean?,
}

local PIDController = {}
PIDController.__index = PIDController

export type PIDController = setmetatable<{
	Kp: number,
	Ki: number,
	Kd: number,
	Setpoint: number,
	OutputMin: number,
	OutputMax: number,
	IntegralMin: number,
	IntegralMax: number,
	DerivativeOnMeasurement: boolean,
	_lastError: number?,
	_lastMeasurement: number?,
	_integral: number,
}, typeof(PIDController)>

function PIDController.new(params: PIDControllerParams?): PIDController
	params = params or {}

	local outputMin = params.OutputMin or DEFAULT_MIN
	local outputMax = params.OutputMax or DEFAULT_MAX
	local integralMin = params.IntegralMin or DEFAULT_MIN
	local integralMax = params.IntegralMax or DEFAULT_MAX
	local initialMeasurement = params.InitialMeasurement
	local setpoint = params.Setpoint

	if setpoint == nil then
		setpoint = initialMeasurement or 0
	end

	assert(outputMin <= outputMax, "OutputMin must be <= OutputMax")
	assert(integralMin <= integralMax, "IntegralMin must be <= IntegralMax")

	local derivativeOnMeasurement: boolean
	if params.DerivativeOnMeasurement == nil then
		derivativeOnMeasurement = true
	else
		derivativeOnMeasurement = params.DerivativeOnMeasurement
	end

	local self = setmetatable({
		Kp = params.Kp or 0,
		Ki = params.Ki or 0,
		Kd = params.Kd or 0,
		Setpoint = setpoint,
		OutputMin = outputMin,
		OutputMax = outputMax,
		IntegralMin = integralMin,
		IntegralMax = integralMax,
		DerivativeOnMeasurement = derivativeOnMeasurement,
		_lastError = nil,
		_lastMeasurement = initialMeasurement,
		_integral = 0,
	}, PIDController)

	return self
end

function PIDController:SetTunings(kp: number, ki: number, kd: number)
	self.Kp = kp
	self.Ki = ki
	self.Kd = kd
end

function PIDController:SetKp(kp: number)
	self.Kp = kp
end

function PIDController:SetKi(ki: number)
	self.Ki = ki
end

function PIDController:SetKd(kd: number)
	self.Kd = kd
end

function PIDController:SetOutputLimits(minValue: number, maxValue: number)
	assert(minValue <= maxValue, "minValue must be <= maxValue")
	self.OutputMin = minValue
	self.OutputMax = maxValue
end

function PIDController:SetIntegralLimits(minValue: number, maxValue: number)
	assert(minValue <= maxValue, "minValue must be <= maxValue")
	self.IntegralMin = minValue
	self.IntegralMax = maxValue
	self._integral = math.clamp(self._integral, minValue, maxValue)
end

function PIDController:Reset(measurement: number?)
	self._lastError = nil
	self._lastMeasurement = measurement
	self._integral = 0
end

function PIDController:Update(dt: number, measurement: number, setpoint: number?): number
	assert(dt > 0, "dt must be > 0")

	if setpoint ~= nil then
		self.Setpoint = setpoint
	end

	local errorValue = self.Setpoint - measurement
	local proportional = self.Kp * errorValue

	local nextIntegral =
		math.clamp(self._integral + (self.Ki * errorValue * dt), self.IntegralMin, self.IntegralMax)

	local derivative = 0
	if self.DerivativeOnMeasurement then
		if self._lastMeasurement then
			derivative = -self.Kd * ((measurement - self._lastMeasurement) / dt)
		end
	else
		if self._lastError then
			derivative = self.Kd * ((errorValue - self._lastError) / dt)
		end
	end

	local unclampedOutput = proportional + nextIntegral + derivative
	local output = math.clamp(unclampedOutput, self.OutputMin, self.OutputMax)

	-- If output is saturated and the current error pushes further into saturation, freeze integration.
	if output ~= unclampedOutput then
		local saturatingHigh = unclampedOutput > self.OutputMax and errorValue > 0
		local saturatingLow = unclampedOutput < self.OutputMin and errorValue < 0
		if saturatingHigh or saturatingLow then
			nextIntegral = self._integral
			output =
				math.clamp(proportional + nextIntegral + derivative, self.OutputMin, self.OutputMax)
		end
	end

	self._integral = nextIntegral
	self._lastError = errorValue
	self._lastMeasurement = measurement

	return output
end

return PIDController
