#include "udp_camera_receiver/udp_camera_receiver.hpp"

#include <gtest/gtest.h>

#include <cstring>

namespace
{
void writeLeFloat(std::vector<uint8_t>& data, size_t offset, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    data[offset] = bits & 0xff;
    data[offset + 1] = (bits >> 8) & 0xff;
    data[offset + 2] = (bits >> 16) & 0xff;
    data[offset + 3] = (bits >> 24) & 0xff;
}
}  // namespace

TEST(BoxParser, ParsesOneObject)
{
    std::vector<uint8_t> payload(115);
    for (size_t i = 0; i < 24; ++i) writeLeFloat(payload, i * 4, static_cast<float>(i) + 0.25F);
    for (size_t i = 0; i < 4; ++i) writeLeFloat(payload, 96 + i * 4, 100.0F + i);
    payload[112] = 'V';
    payload[113] = 'E';
    payload[114] = ' ';

    std::vector<udp_camera_receiver::BoxObject> objects;
    std::string error;
    ASSERT_TRUE(udp_camera_receiver::parseBoxPayload(payload.data(), payload.size(), objects, error));
    ASSERT_EQ(objects.size(), 1U);
    EXPECT_FLOAT_EQ(objects[0].corners_3d[0], 0.25F);
    EXPECT_FLOAT_EQ(objects[0].corners_3d[23], 23.25F);
    EXPECT_EQ(objects[0].bbox_2d_raw, (std::array<float, 4>{100.0F, 101.0F, 102.0F, 103.0F}));
    EXPECT_EQ(objects[0].class_tag, "VE");
}

TEST(BoxParser, RejectsInvalidSize)
{
    std::vector<uint8_t> payload(114);
    std::vector<udp_camera_receiver::BoxObject> objects;
    std::string error;
    EXPECT_FALSE(udp_camera_receiver::parseBoxPayload(payload.data(), payload.size(), objects, error));
}

TEST(BoxParser, MapsMoraiSemanticTags)
{
    EXPECT_EQ(udp_camera_receiver::boxClassLabel(std::string("\xff\x02\x02", 3)), "Vehicle");
    EXPECT_EQ(udp_camera_receiver::boxClassLabel(std::string("\x62\x02\xff", 3)), "Pedestrian");
    EXPECT_EQ(udp_camera_receiver::boxClassLabel(std::string("\xec\xff\x02", 3)), "Obstacle");
    EXPECT_EQ(udp_camera_receiver::boxClassLabel("bad"), "Unknown");
}
