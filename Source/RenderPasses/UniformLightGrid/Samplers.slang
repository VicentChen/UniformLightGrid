__exported import Utils.Sampling.SampleGenerator;

void sampleDistanceUniformly<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    float deltaD = dmax - dmin;
    if (deltaD < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    float x = sampleNext1D(sg);
    d = dmin + x * deltaD;
    pdf = 1 / deltaD;
}

void sampleDistanceByDist<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    if (dmax - dmin < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    // sample with CDF
    float c = 1 - (log(dmax) - log(dmin));
    float x = sampleNext1D(sg);
    d = c / x;
    pdf = c / d;
}

void sampleDistanceByDistSquare<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    if (dmax - dmin < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    // sample with CDF
    float c = dmin * dmax / (dmax - dmin);
    float x = sampleNext1D(sg);
    d = sqrt(c / x);
    pdf = c / (d * d);
}

void sampleDistanceByDistQuad<S : ISampleGenerator>(const float dmin, const float dmax, inout S sg, out float d, out float pdf)
{
    // avoid dmin == dmax
    if (dmax - dmin < 1E-5)
    {
        d = 0;
        pdf = 1;
        return;
    }

    // sample with CDF
    float dmin3 = pow(dmin, 3);
    float dmax3 = pow(dmax, 3);
    float c = 3 * (dmin3 * dmax3) / (dmax3 - dmin3);
    float x = sampleNext1D(sg);
    d = sqrt(c / x);
    pdf = c / pow(d, 4);
}

void sampleRangeUniformly<S : ISampleGenerator>(const uint2 range, inout S sg, out uint idx, out float pdf)
{
    float delta = float(range.y) - float(range.x) + 1;
    float x = sampleNext1D(sg) * 0.999999f; // FIXME: hack to avoid x == 1
    idx = floor(range.x + x * delta);
    pdf = 1.f / delta;
}
