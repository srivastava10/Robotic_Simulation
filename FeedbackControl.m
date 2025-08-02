function [V,integralError,derivativeError,filteredDerivativeError] = FeedbackControl(X,Xd,Xd_next,Kp,Ki,Kd,delta_t,integralError,derivativeError,filteredDerivativeError,alpha)
    
    Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd)); % Error Twist

    integralError = integralError + Xerr*delta_t;
     % Derivative term - calculate rate of change of error
    currentDerivativeError = (Xerr - derivativeError) / delta_t;
    
    % Apply low-pass filter to derivative term to reduce noise
    % First-order low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    filteredDerivativeError = alpha * currentDerivativeError + (1 - alpha) * filteredDerivativeError;
    
    % Update derivative error for next iteration
    derivativeError = Xerr;
    
    Adj = Adjoint(TransInv(X)*Xd);

    Vd = se3ToVec((1/delta_t)*MatrixLog6(TransInv(Xd)*Xd_next));

    V = Adj*Vd+Kp*Xerr + Ki*integralError+ Kd*filteredDerivativeError; %Feedback Control Law

    max_integral = 0.2;
    integralError = max(-max_integral, min(max_integral, integralError)); %To prevent integralError from growing too large

end

