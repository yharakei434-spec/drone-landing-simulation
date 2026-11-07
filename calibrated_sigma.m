function sigma=calibrated_sigma(features,model)
% Frozen offline model; features contain pixels/estimated residuals only.
z=[1 (features-model.mean(:)')./model.std(:)'];lv=z*model.coefficients(:);variance=exp(min(log(model.sigma_cap_m^2),max(log(model.sigma_floor_m^2),lv)))*model.variance_scale;
sigma=sqrt(min(model.sigma_cap_m^2,max(model.sigma_floor_m^2,variance)));
end
