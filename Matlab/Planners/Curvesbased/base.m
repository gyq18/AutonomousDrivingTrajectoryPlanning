function res = base(n,k,t)
    res = nchoosek(n,k)*t^k*(1-t)^(n-k);
end

