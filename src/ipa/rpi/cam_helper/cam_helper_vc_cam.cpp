/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * camera helper for imx219 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <libcamera/base/log.h>

#include <string>
#include <cmath>


#include "cam_helper.h"


using namespace RPiController;
using namespace libcamera;
LOG_DEFINE_CATEGORY(CamHelperImxVCCamera)



/*
 * We care about one gain register and a pair of exposure registers. Their I2C
 * addresses from the Sony IMX219 datasheet:
 */
constexpr uint32_t gainReg = 0x157;
constexpr uint32_t expHiReg = 0x010A;
constexpr uint32_t expLoReg = 0x109;
constexpr uint32_t frameLengthHiReg = 0x160;
constexpr uint32_t frameLengthLoReg = 0x161;
constexpr uint32_t lineLengthHiReg = 0x162;
constexpr uint32_t lineLengthLoReg = 0x163;
constexpr std::initializer_list<uint32_t> registerList [[maybe_unused]]
	= { expHiReg, expLoReg, gainReg, frameLengthHiReg, frameLengthLoReg,
	    lineLengthHiReg, lineLengthLoReg };

class CamHelperImxVCCamera : public CamHelper
{

public:
	CamHelperImxVCCamera();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	unsigned int mistrustFramesModeSwitch() const override;
	bool sensorEmbeddedDataPresent() const override;

	std::pair<uint32_t, uint32_t>
    getBlanking(libcamera::utils::Duration &exposure,
                libcamera::utils::Duration minFrameDuration,
                libcamera::utils::Duration maxFrameDuration) const override;

private:
	/*
	 * Smallest difference between the frame length and integration time,
	 * in units of lines.
	 */
	static constexpr int frameIntegrationDiff = 4;

	// void populateMetadata(const MdParser::RegisterMap &registers,
	// 		      Metadata &metadata) const override;
};

CamHelperImxVCCamera::CamHelperImxVCCamera()
#if ENABLE_EMBEDDED_DATA
	: CamHelper(std::make_unique<MdParserSmia>(registerList), frameIntegrationDiff)
#else
	: CamHelper({}, frameIntegrationDiff)
#endif
{
}

uint32_t CamHelperImxVCCamera::gainCode(double gain) const
{
	// Convert linear gain to milli-dB: mDb = round(20000 * log10(gain))
	// Clamp gain to a tiny positive to avoid log10(0)


	double g = std::max(gain, 1e-9);
	long mdb = std::lround(20000.0 * std::log10(g));
	// Avoid negative underflow into uint32_t
	if (mdb < 0)
		mdb = 0;
	LOG(CamHelperImxVCCamera, Debug) << "Gain:  " << gain 
				<< " GainCode: " << mdb << " dB";
	return static_cast<uint32_t>(mdb);
}

double CamHelperImxVCCamera::gain(uint32_t gainCode) const
{
	// Convert milli-dB to linear gain: gain = 10^(mDb/20000)
	return std::pow(10.0, static_cast<double>(gainCode) / 20000.0);
}

unsigned int CamHelperImxVCCamera::mistrustFramesModeSwitch() const
{
	/*
	 * For reasons unknown, we do occasionally get a bogus metadata frame
	 * at a mode switch (though not at start-up). Possibly warrants some
	 * investigation, though not a big deal.
	 */
	return 1;
}

bool CamHelperImxVCCamera::sensorEmbeddedDataPresent() const
{
	return false;
}

using libcamera::utils::Duration;
std::pair<uint32_t, uint32_t>
CamHelperImxVCCamera::getBlanking(Duration &exposure,
                                  Duration minFrameDuration,
                                  Duration maxFrameDuration) const
{
    // minFrameDuration == maxFrameDuration
    static_cast<void>(maxFrameDuration);

    // IMX900 (4-lane) base hmax values (hblank = 0)
	constexpr uint32_t numLanes = 4;
    struct Entry { uint32_t bitDepth; uint32_t hmaxBase; };
    static constexpr Entry kImx900_4Lane[] = {
        {  8, 338 },
        { 10, 364 },
        { 12, 610 },
    };

    const uint32_t bitDepth = mode_.bitdepth;

    uint32_t hmaxBase = mode_.width;
    for (const auto &e : kImx900_4Lane) {
        if (e.bitDepth == bitDepth) {
            hmaxBase = e.hmaxBase;
            break;
        }
    }

    const uint64_t sensorPixelRate = mode_.pixelRate / (2 * numLanes);

    Duration lineLength = mode_.minLineLength;
    uint32_t frameLengthLines = minFrameDuration / mode_.minLineLength;

    if (frameLengthLines > mode_.maxFrameLength) {
        Duration lineLengthAdjusted = lineLength * frameLengthLines / mode_.maxFrameLength;
        lineLength = std::min(mode_.maxLineLength, lineLengthAdjusted);
        frameLengthLines = mode_.maxFrameLength;
    }

    const uint64_t lineNs = lineLength.get<std::nano>();
    const uint32_t hmaxOverwrite =
        static_cast<uint32_t>((lineNs * sensorPixelRate) / 1000000000ULL);

    const uint32_t hblank =
        (hmaxOverwrite > hmaxBase)
            ? (hmaxOverwrite - hmaxBase) * numLanes
            : 0;

    const uint32_t vblank =
        (frameLengthLines > mode_.height)
            ? (frameLengthLines - mode_.height)
            : 0;

    const uint32_t exposureLines =
        std::min(frameLengthLines - frameIntegrationDiff,
                 CamHelper::exposureLines(exposure, lineLength));
    exposure = CamHelper::exposure(exposureLines, lineLength);

    return { vblank, hblank };
}

static CamHelper *create()
{
	return new CamHelperImxVCCamera();
}

static RegisterCamHelper reg("vc_mipi_camera", &create);
