% quick check of the B-loss data against published dispatches (Case 2 literature)
s = eld6_data();
P = [447.5840 173.2010 263.3310 138.8520 165.3280 87.1500;      % IPSO-TVAC  loss 12.4460 cost 15443.063
     447.4021 173.2407 263.3812 138.9774 165.3897 87.0538;      % HCRO-DE    loss 12.4449 cost 15443.075
     447.4970 173.3221 263.4745 139.0594 165.4761 87.1280];     % IPSO       loss 12.9584 cost 15449.882
for k = 1:size(P,1)
    p = P(k,:);
    L = p*s.B*p' + s.B0*p' + s.B00;
    F = sum(s.a + s.b.*p + s.c.*p.^2);
    fprintf('sum P = %.4f  loss = %.4f  sumP-PD-L = %.4f  cost = %.3f\n', sum(p), L, sum(p)-s.PD-L, F);
end
