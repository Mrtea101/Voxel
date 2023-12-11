

#include "Noise.h"

double FNoise::ComputeNoise(const FVector& InLocation, const FNoiseSettings& InNoiseSettings)
{
    ENoiseType type = InNoiseSettings.Type;
    double freq = InNoiseSettings.Frequency;
    double ampl = InNoiseSettings.Amplitude;
    int octave = InNoiseSettings.OctaveCount;

    return octave > 1 ?
        ComputeNoiseWithOctaves(InLocation * freq, type, octave) * ampl :
        ComputeNoise(InLocation * freq, type) * ampl;
}

double FNoise::ComputeNoise(const FVector& InLocation, ENoiseType InNoiseType)
{
    //TODO: Implement other noise types
    //https://github.com/Auburn/FastNoiseLite/blob/master/Cpp/FastNoiseLite.h
    // line 1613 for cellular

    switch (InNoiseType)
    {
        case ENoiseType::Perlin:
            return (FMath::PerlinNoise3D(InLocation) + 1) / 2;   //[-1,1] -> [0,1]
       // case ENoiseType::Cellular:
       //     return noise.cellular(point).x;
       // case ENoiseType::Simplex:
       //     return (noise.snoise(point) + 1) / 2;    //[-1,1] -> [0,1]
        default:
            return 0.0f;
    }
}

double FNoise::ComputeNoiseWithOctaves(const FVector& InLocation, ENoiseType InNoiseType, int32 InOctaveCount)
{
    double persistence = 0.5f;
    double ampl = 1;
    double freq = 1;

    double totalAmpl = 0.0f;
    double result = 0.0f;

    for (int i = 0; i < InOctaveCount; i++)
    {
        result += ComputeNoise(freq * InLocation, InNoiseType) * ampl;
        totalAmpl += ampl;

        ampl *= persistence;
        freq *= 2; //lacunarity
    }

    return result / totalAmpl;
}
