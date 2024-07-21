#include <math.h>

const float sin_table[360] = {
     0.00000000000000f,  0.01745240643728f,  0.03489949670250f,  0.05233595624294f,  0.06975647374413f,  0.08715574274766f,  0.10452846326765f,  0.12186934340515f,  0.13917310096007f,  0.15643446504023f,
     0.17364817766693f,  0.19080899537654f,  0.20791169081776f,  0.22495105434387f,  0.24192189559967f,  0.25881904510252f,  0.27563735581700f,  0.29237170472274f,  0.30901699437495f,  0.32556815445716f,
     0.34202014332567f,  0.35836794954530f,  0.37460659341591f,  0.39073112848927f,  0.40673664307580f,  0.42261826174070f,  0.43837114678908f,  0.45399049973955f,  0.46947156278589f,  0.48480962024634f,
     0.50000000000000f,  0.51503807491005f,  0.52991926423320f,  0.54463903501503f,  0.55919290347075f,  0.57357643635105f,  0.58778525229247f,  0.60181502315205f,  0.61566147532566f,  0.62932039104984f,
     0.64278760968654f,  0.65605902899051f,  0.66913060635886f,  0.68199836006250f,  0.69465837045900f,  0.70710678118655f,  0.71933980033865f,  0.73135370161917f,  0.74314482547739f,  0.75470958022277f,
     0.76604444311898f,  0.77714596145697f,  0.78801075360672f,  0.79863551004729f,  0.80901699437495f,  0.81915204428899f,  0.82903757255504f,  0.83867056794542f,  0.84804809615643f,  0.85716730070211f,
     0.86602540378444f,  0.87461970713940f,  0.88294759285893f,  0.89100652418837f,  0.89879404629917f,  0.90630778703665f,  0.91354545764260f,  0.92050485345244f,  0.92718385456679f,  0.93358042649720f,
     0.93969262078591f,  0.94551857559932f,  0.95105651629515f,  0.95630475596304f,  0.96126169593832f,  0.96592582628907f,  0.97029572627600f,  0.97437006478524f,  0.97814760073381f,  0.98162718344766f,
     0.98480775301221f,  0.98768834059514f,  0.99026806874157f,  0.99254615164132f,  0.99452189536827f,  0.99619469809175f,  0.99756405025982f,  0.99862953475457f,  0.99939082701910f,  0.99984769515639f,
     1.00000000000000f,  0.99984769515639f,  0.99939082701910f,  0.99862953475457f,  0.99756405025982f,  0.99619469809175f,  0.99452189536827f,  0.99254615164132f,  0.99026806874157f,  0.98768834059514f,
     0.98480775301221f,  0.98162718344766f,  0.97814760073381f,  0.97437006478524f,  0.97029572627600f,  0.96592582628907f,  0.96126169593832f,  0.95630475596304f,  0.95105651629515f,  0.94551857559932f,
     0.93969262078591f,  0.93358042649720f,  0.92718385456679f,  0.92050485345244f,  0.91354545764260f,  0.90630778703665f,  0.89879404629917f,  0.89100652418837f,  0.88294759285893f,  0.87461970713940f,
     0.86602540378444f,  0.85716730070211f,  0.84804809615643f,  0.83867056794542f,  0.82903757255504f,  0.81915204428899f,  0.80901699437495f,  0.79863551004729f,  0.78801075360672f,  0.77714596145697f,
     0.76604444311898f,  0.75470958022277f,  0.74314482547739f,  0.73135370161917f,  0.71933980033865f,  0.70710678118655f,  0.69465837045900f,  0.68199836006250f,  0.66913060635886f,  0.65605902899051f,
     0.64278760968654f,  0.62932039104984f,  0.61566147532566f,  0.60181502315205f,  0.58778525229247f,  0.57357643635105f,  0.55919290347075f,  0.54463903501503f,  0.52991926423320f,  0.51503807491005f,
     0.50000000000000f,  0.48480962024634f,  0.46947156278589f,  0.45399049973955f,  0.43837114678908f,  0.42261826174070f,  0.40673664307580f,  0.39073112848927f,  0.37460659341591f,  0.35836794954530f,
     0.34202014332567f,  0.32556815445716f,  0.30901699437495f,  0.29237170472274f,  0.27563735581700f,  0.25881904510252f,  0.24192189559967f,  0.22495105434387f,  0.20791169081776f,  0.19080899537654f,
     0.17364817766693f,  0.15643446504023f,  0.13917310096007f,  0.12186934340515f,  0.10452846326765f,  0.08715574274766f,  0.06975647374413f,  0.05233595624294f,  0.03489949670250f,  0.01745240643728f,
     0.00000000000000f, -0.01745240643728f, -0.03489949670250f, -0.05233595624294f, -0.06975647374413f, -0.08715574274766f, -0.10452846326765f, -0.12186934340515f, -0.13917310096007f, -0.15643446504023f,
    -0.17364817766693f, -0.19080899537654f, -0.20791169081776f, -0.22495105434386f, -0.24192189559967f, -0.25881904510252f, -0.27563735581700f, -0.29237170472274f, -0.30901699437495f, -0.32556815445716f,
    -0.34202014332567f, -0.35836794954530f, -0.37460659341591f, -0.39073112848927f, -0.40673664307580f, -0.42261826174070f, -0.43837114678908f, -0.45399049973955f, -0.46947156278589f, -0.48480962024634f,
    -0.50000000000000f, -0.51503807491005f, -0.52991926423320f, -0.54463903501503f, -0.55919290347075f, -0.57357643635105f, -0.58778525229247f, -0.60181502315205f, -0.61566147532566f, -0.62932039104984f,
    -0.64278760968654f, -0.65605902899051f, -0.66913060635886f, -0.68199836006250f, -0.69465837045900f, -0.70710678118655f, -0.71933980033865f, -0.73135370161917f, -0.74314482547739f, -0.75470958022277f,
    -0.76604444311898f, -0.77714596145697f, -0.78801075360672f, -0.79863551004729f, -0.80901699437495f, -0.81915204428899f, -0.82903757255504f, -0.83867056794542f, -0.84804809615643f, -0.85716730070211f,
    -0.86602540378444f, -0.87461970713940f, -0.88294759285893f, -0.89100652418837f, -0.89879404629917f, -0.90630778703665f, -0.91354545764260f, -0.92050485345244f, -0.92718385456679f, -0.93358042649720f,
    -0.93969262078591f, -0.94551857559932f, -0.95105651629515f, -0.95630475596304f, -0.96126169593832f, -0.96592582628907f, -0.97029572627600f, -0.97437006478524f, -0.97814760073381f, -0.98162718344766f,
    -0.98480775301221f, -0.98768834059514f, -0.99026806874157f, -0.99254615164132f, -0.99452189536827f, -0.99619469809175f, -0.99756405025982f, -0.99862953475457f, -0.99939082701910f, -0.99984769515639f,
    -1.00000000000000f, -0.99984769515639f, -0.99939082701910f, -0.99862953475457f, -0.99756405025982f, -0.99619469809175f, -0.99452189536827f, -0.99254615164132f, -0.99026806874157f, -0.98768834059514f,
    -0.98480775301221f, -0.98162718344766f, -0.97814760073381f, -0.97437006478524f, -0.97029572627600f, -0.96592582628907f, -0.96126169593832f, -0.95630475596304f, -0.95105651629515f, -0.94551857559932f,
    -0.93969262078591f, -0.93358042649720f, -0.92718385456679f, -0.92050485345244f, -0.91354545764260f, -0.90630778703665f, -0.89879404629917f, -0.89100652418837f, -0.88294759285893f, -0.87461970713940f,
    -0.86602540378444f, -0.85716730070211f, -0.84804809615643f, -0.83867056794542f, -0.82903757255504f, -0.81915204428899f, -0.80901699437495f, -0.79863551004729f, -0.78801075360672f, -0.77714596145697f,
    -0.76604444311898f, -0.75470958022277f, -0.74314482547739f, -0.73135370161917f, -0.71933980033865f, -0.70710678118655f, -0.69465837045900f, -0.68199836006250f, -0.66913060635886f, -0.65605902899051f,
    -0.64278760968654f, -0.62932039104984f, -0.61566147532566f, -0.60181502315205f, -0.58778525229247f, -0.57357643635105f, -0.55919290347075f, -0.54463903501503f, -0.52991926423321f, -0.51503807491005f,
    -0.50000000000000f, -0.48480962024634f, -0.46947156278589f, -0.45399049973955f, -0.43837114678908f, -0.42261826174070f, -0.40673664307580f, -0.39073112848927f, -0.37460659341591f, -0.35836794954530f,
    -0.34202014332567f, -0.32556815445716f, -0.30901699437495f, -0.29237170472274f, -0.27563735581700f, -0.25881904510252f, -0.24192189559967f, -0.22495105434387f, -0.20791169081776f, -0.19080899537654f,
    -0.17364817766693f, -0.15643446504023f, -0.13917310096007f, -0.12186934340515f, -0.10452846326765f, -0.08715574274766f, -0.06975647374413f, -0.05233595624294f, -0.03489949670250f, -0.01745240643728f
};

// 从正弦表中取值并进行线性插值
float fast_sin(float x) {
    x = fmodf(x, 360);

    int index = (int)x;
    float frac = x - index;

    return sin_table[index] + frac * (sin_table[(index + 1) % 360] - sin_table[index]);
}

// 从正弦表中取值但不进行插值
float ffast_sin(int32_t x) {
    x = x % 360;
    return sin_table[x];
}

// 从正弦表计算余弦值并进行线性插值
float fast_cos(float x) {
    x += 90;
    x = fmodf(x, 360);

    int index = (int)x;
    float frac = x - index;

    return sin_table[index] + frac * (sin_table[(index + 1) % 360] - sin_table[index]);
}

// 从正弦表计算余弦值但不进行插值
float ffast_cos(int32_t x) {
    x += 90;
    x = x % 360;
    return sin_table[x];
}

// 计算切线值并进行线性插值
float fast_tan(float x) {
    return fast_sin(x) / fast_cos(x);
}

// 计算切线值但不进行插值
float ffast_tan(int32_t x) {
    return ffast_sin(x) / ffast_cos(x);
}