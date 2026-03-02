--!native

local DEFAULT_MIN = -math.huge
local DEFAULT_MAX = math.huge

export type DerivativeMode = "Measurement" | "Error"

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
	DerivativeMode: DerivativeMode?,
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
	DerivativeMode: DerivativeMode,
	_lastError: number?,
	_lastMeasurement: number?,
	_integral: number,
	_hasOutputLimits: boolean,
	_hasIntegralLimits: boolean,
	_updateImpl: (PIDController, number, number) -> number,
}, typeof(PIDController)>

local function updateDerivativeOnMeasurement(
	self: PIDController,
	dt: number,
	measurement: number
): number
	local setpoint = self.Setpoint
	local errorValue = setpoint - measurement

	local kp = self.Kp
	local ki = self.Ki
	local kd = self.Kd
	local integral = self._integral
	local invDt = 1 / dt

	local nextIntegral = integral + (ki * errorValue * dt)
	if self._hasIntegralLimits then
		local integralMin = self.IntegralMin
		local integralMax = self.IntegralMax
		if nextIntegral > integralMax then
			nextIntegral = integralMax
		elseif nextIntegral < integralMin then
			nextIntegral = integralMin
		end
	end

	local derivative = 0
	local lastMeasurement = self._lastMeasurement
	if lastMeasurement then
		derivative = -kd * (measurement - lastMeasurement) * invDt
	end

	local unclampedOutput = (kp * errorValue) + nextIntegral + derivative
	local output = unclampedOutput
	if self._hasOutputLimits then
		local outputMin = self.OutputMin
		local outputMax = self.OutputMax
		if output > outputMax then
			output = outputMax
		elseif output < outputMin then
			output = outputMin
		end

		-- If output is saturated and the current error pushes further into saturation, freeze integration.
		if output ~= unclampedOutput then
			local saturatingHigh = unclampedOutput > outputMax and errorValue > 0
			local saturatingLow = unclampedOutput < outputMin and errorValue < 0
			if saturatingHigh or saturatingLow then
				nextIntegral = integral
				output = (kp * errorValue) + nextIntegral + derivative
				if output > outputMax then
					output = outputMax
				elseif output < outputMin then
					output = outputMin
				end
			end
		end
	end

	self._integral = nextIntegral
	self._lastError = errorValue
	self._lastMeasurement = measurement

	return output
end

local function updateDerivativeOnError(self: PIDController, dt: number, measurement: number): number
	local setpoint = self.Setpoint
	local errorValue = setpoint - measurement

	local kp = self.Kp
	local ki = self.Ki
	local kd = self.Kd
	local integral = self._integral
	local invDt = 1 / dt

	local nextIntegral = integral + (ki * errorValue * dt)
	if self._hasIntegralLimits then
		local integralMin = self.IntegralMin
		local integralMax = self.IntegralMax
		if nextIntegral > integralMax then
			nextIntegral = integralMax
		elseif nextIntegral < integralMin then
			nextIntegral = integralMin
		end
	end

	local derivative = 0
	local lastError = self._lastError
	if lastError then
		derivative = kd * (errorValue - lastError) * invDt
	end

	local unclampedOutput = (kp * errorValue) + nextIntegral + derivative
	local output = unclampedOutput
	if self._hasOutputLimits then
		local outputMin = self.OutputMin
		local outputMax = self.OutputMax
		if output > outputMax then
			output = outputMax
		elseif output < outputMin then
			output = outputMin
		end

		-- If output is saturated and the current error pushes further into saturation, freeze integration.
		if output ~= unclampedOutput then
			local saturatingHigh = unclampedOutput > outputMax and errorValue > 0
			local saturatingLow = unclampedOutput < outputMin and errorValue < 0
			if saturatingHigh or saturatingLow then
				nextIntegral = integral
				output = (kp * errorValue) + nextIntegral + derivative
				if output > outputMax then
					output = outputMax
				elseif output < outputMin then
					output = outputMin
				end
			end
		end
	end

	self._integral = nextIntegral
	self._lastError = errorValue
	self._lastMeasurement = measurement

	return output
end

local function updateDerivativeOnMeasurementNoIntegral(
	self: PIDController,
	dt: number,
	measurement: number
): number
	local setpoint = self.Setpoint
	local errorValue = setpoint - measurement

	local kp = self.Kp
	local kd = self.Kd
	local integral = self._integral
	local invDt = 1 / dt

	local derivative = 0
	local lastMeasurement = self._lastMeasurement
	if lastMeasurement then
		derivative = -kd * (measurement - lastMeasurement) * invDt
	end

	local output = (kp * errorValue) + integral + derivative
	if self._hasOutputLimits then
		local outputMin = self.OutputMin
		local outputMax = self.OutputMax
		if output > outputMax then
			output = outputMax
		elseif output < outputMin then
			output = outputMin
		end
	end

	self._lastError = errorValue
	self._lastMeasurement = measurement

	return output
end

local function updateDerivativeOnErrorNoIntegral(self: PIDController, dt: number, measurement: number): number
	local setpoint = self.Setpoint
	local errorValue = setpoint - measurement

	local kp = self.Kp
	local kd = self.Kd
	local integral = self._integral
	local invDt = 1 / dt

	local derivative = 0
	local lastError = self._lastError
	if lastError then
		derivative = kd * (errorValue - lastError) * invDt
	end

	local output = (kp * errorValue) + integral + derivative
	if self._hasOutputLimits then
		local outputMin = self.OutputMin
		local outputMax = self.OutputMax
		if output > outputMax then
			output = outputMax
		elseif output < outputMin then
			output = outputMin
		end
	end

	self._lastError = errorValue
	self._lastMeasurement = measurement

	return output
end

local function selectUpdateImpl(mode: DerivativeMode, ki: number): (PIDController, number, number) -> number
	if mode == "Measurement" then
		if ki == 0 then
			return updateDerivativeOnMeasurementNoIntegral
		end
		return updateDerivativeOnMeasurement
	end

	if ki == 0 then
		return updateDerivativeOnErrorNoIntegral
	end
	return updateDerivativeOnError
end

function PIDController.new(params: PIDControllerParams?): PIDController
	params = params or {}

	local hasOutputLimits = params.OutputMin ~= nil or params.OutputMax ~= nil
	local hasIntegralLimits = params.IntegralMin ~= nil or params.IntegralMax ~= nil
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

	local derivativeMode = params.DerivativeMode or "Measurement"
	assert(
		derivativeMode == "Measurement" or derivativeMode == "Error",
		'DerivativeMode must be "Measurement" or "Error"'
	)

	local kp = params.Kp or 0
	local ki = params.Ki or 0
	local kd = params.Kd or 0

	local self = setmetatable({
		Kp = kp,
		Ki = ki,
		Kd = kd,
		Setpoint = setpoint,
		OutputMin = outputMin,
		OutputMax = outputMax,
		IntegralMin = integralMin,
		IntegralMax = integralMax,
		DerivativeMode = derivativeMode,
		_lastError = nil,
		_lastMeasurement = initialMeasurement,
		_integral = 0,
		_hasOutputLimits = hasOutputLimits,
		_hasIntegralLimits = hasIntegralLimits,
		_updateImpl = selectUpdateImpl(derivativeMode, ki),
	}, PIDController)

	return self
end

function PIDController:SetTunings(kp: number, ki: number, kd: number)
	self.Kp = kp
	self.Ki = ki
	self.Kd = kd
	self._updateImpl = selectUpdateImpl(self.DerivativeMode, ki)
end

function PIDController:SetKp(kp: number)
	self.Kp = kp
end

function PIDController:SetKi(ki: number)
	self.Ki = ki
	self._updateImpl = selectUpdateImpl(self.DerivativeMode, ki)
end

function PIDController:SetKd(kd: number)
	self.Kd = kd
end

function PIDController:SetDerivativeMode(mode: DerivativeMode)
	assert(mode == "Measurement" or mode == "Error", 'mode must be "Measurement" or "Error"')
	self.DerivativeMode = mode
	self._updateImpl = selectUpdateImpl(mode, self.Ki)
end

function PIDController:SetSetpoint(setpoint: number)
	self.Setpoint = setpoint
end

function PIDController:SetOutputLimits(minValue: number, maxValue: number)
	assert(minValue <= maxValue, "minValue must be <= maxValue")
	self.OutputMin = minValue
	self.OutputMax = maxValue
	self._hasOutputLimits = true
end

function PIDController:SetIntegralLimits(minValue: number, maxValue: number)
	assert(minValue <= maxValue, "minValue must be <= maxValue")
	self.IntegralMin = minValue
	self.IntegralMax = maxValue
	self._hasIntegralLimits = true
	self._integral = math.clamp(self._integral, minValue, maxValue)
end

function PIDController:ClearOutputLimits()
	self.OutputMin = DEFAULT_MIN
	self.OutputMax = DEFAULT_MAX
	self._hasOutputLimits = false
end

function PIDController:ClearIntegralLimits()
	self.IntegralMin = DEFAULT_MIN
	self.IntegralMax = DEFAULT_MAX
	self._hasIntegralLimits = false
end

function PIDController:Reset(measurement: number?)
	self._lastError = nil
	self._lastMeasurement = measurement
	self._integral = 0
end

function PIDController:GetMeasurement(): number?
	return self._lastMeasurement
end

function PIDController:Update(dt: number, measurement: number): number
	return self._updateImpl(self, dt, measurement)
end

return PIDController
