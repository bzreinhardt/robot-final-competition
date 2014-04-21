function point = haltonSample(index,base)
%HALTONSAMPLE finds the index-th number in the halton sequence with the
%given base
%INPUTS
%index - int >= 1
%base - int >= 1
result = 0;
f = 1/base;
i = index;
while i>0
    result = result + f * mod(i,base);
    i = floor(i/base);
    f = f/base;
end
point = result;
end