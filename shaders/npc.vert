#version 460

// Instanced people, animated here on the GPU so the CPU only sends a few numbers per person.
//
// The body is a handful of rigid boxes (see Meshes.cpp). Each vertex is moved by a small
// chain of rotations: the limb around its joint, the upper body around the hips, then the
// whole person around its feet. Walking swings the legs (with a knee lift), arms, hips and
// shoulders; standing people breathe, shift their weight and play an idle gesture that the
// simulation picks (looking around, or talking with their hands).

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in float inPart;            // body part id, see BodyPart in Meshes.h
layout(location = 3) in vec4 inPositionHeading;  // xyz: feet position, w: heading in radians
layout(location = 4) in vec4 inColorHeight;      // rgb: clothing colour, w: height scale
// x: walk phase in radians, y: stride 0..1,
// z: gesture kind in the integer part, the person's variation 0..1 in the fraction,
// w: gesture weight in 64ths in the integer part, the gesture clock 0..1 in the fraction.
// The packing is written by MakeNpcInstance in src/world/crowd/CrowdVisibility.cpp.
layout(location = 5) in vec4 inAnimation;

layout(push_constant) uniform PushConstants {
    mat4 viewProjection;
} push;

layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) out vec3 outNormal;
layout(location = 2) out vec3 outAlbedo;
layout(location = 3) out vec3 outSurface;        // x: roughness, y: material id, z: seed
layout(location = 4) out vec3 outLocalPosition;  // position on the body, metres above the feet
layout(location = 5) out vec3 outBoxSize;        // unused for people
layout(location = 6) out vec3 outLocalNormal;    // the same as outNormal for people

const int PART_HEAD = 1;
const int PART_LEFT_LEG = 2;
const int PART_RIGHT_LEG = 3;
const int PART_LEFT_ARM = 4;
const int PART_RIGHT_ARM = 5;

const int GESTURE_LOOK_AROUND = 1;
const int GESTURE_TALK = 2;

const float MATERIAL_CHARACTER = 3.0;
const float TWO_PI = 6.28318531;

// Joints, in metres above the feet. The person faces +Z and their left side is +X.
const vec3 HIP = vec3(0.0, 0.9, 0.0);
const float KNEE_HEIGHT = 0.45;
const vec3 LEFT_SHOULDER = vec3(0.25, 1.45, 0.0);
const vec3 RIGHT_SHOULDER = vec3(-0.25, 1.45, 0.0);
const vec3 NECK = vec3(0.0, 1.5, 0.0);

// A positive angle swings a hanging limb backwards and tips an upright one forwards.
vec3 RotateAroundX(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x, v.y * c - v.z * s, v.y * s + v.z * c);
}

// Turns a vector around the vertical axis. An angle of 0 keeps +Z pointing along +Z, and a
// positive angle turns the person to their left.
vec3 RotateAroundY(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x * c + v.z * s, v.y, -v.x * s + v.z * c);
}

// A positive angle lifts a hanging limb out towards +X.
vec3 RotateAroundZ(vec3 v, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(v.x * c - v.y * s, v.x * s + v.y * c, v.z);
}

float Hash(float n) {
    return fract(sin(n) * 43758.5453);
}

// A smooth wave over the gesture clock. The clock wraps from 1 back to 0, so every wave
// uses a whole number of cycles per turn to stay continuous across the wrap.
float Wave(float clock, float cycles, float offset) {
    return sin(TWO_PI * (clock * cycles + offset));
}

struct Pose {
    float legSwing;       // radians, positive swings the left leg back and the right leg forward
    float leftKnee;       // radians the lower legs bend back while they swing forward
    float rightKnee;
    float leftArmPitch;   // radians, positive swings the arm back
    float rightArmPitch;
    float leftArmRoll;    // radians the arms lift out to the sides
    float rightArmRoll;
    float bodyLean;       // radians the upper body tips forward from the hips
    float bodyTwist;      // radians the upper body turns left
    float headTurn;       // radians the head turns left
    float headNod;        // radians the head tips forward
    float sideSway;       // metres the whole body shifts to its left
    float bounce;         // metres the whole body rises
    float breath;         // metres the chest rises
};

Pose WalkPose(float phase, float stride, float variation) {
    // Each person has their own gait: some swing their arms more, some lean into the walk.
    float armSwing = mix(0.55, 0.95, variation);
    float lean = mix(0.02, 0.09, fract(variation * 7.31));

    float swing = sin(phase) * 0.5 * stride;  // about 30 degrees at full stride
    // A leg swinging forward lifts its foot by bending the knee.
    float kneeBend = 0.55 * stride;

    Pose pose;
    pose.legSwing = swing;
    pose.leftKnee = max(0.0, -cos(phase)) * kneeBend;
    pose.rightKnee = max(0.0, cos(phase)) * kneeBend;
    // Each arm swings opposite to the leg on the same side, as in a natural walk.
    pose.leftArmPitch = -swing * armSwing;
    pose.rightArmPitch = swing * armSwing;
    pose.leftArmRoll = 0.04;
    pose.rightArmRoll = -0.04;
    pose.bodyLean = lean * stride;
    // The shoulders turn against the hips, following the forward arm.
    pose.bodyTwist = -swing * 0.25;
    pose.headTurn = 0.0;
    pose.headNod = 0.0;
    // The weight shifts over the foot on the ground, and the body is highest as the legs pass.
    pose.sideSway = sin(phase) * 0.025 * stride;
    pose.bounce = abs(cos(phase)) * 0.035 * stride;
    pose.breath = 0.0;
    return pose;
}

// Standing still: breathing and a slow shift of weight from one foot to the other.
void AddIdle(inout Pose pose, float clock, float variation, float amount) {
    pose.breath += Wave(clock, 2.0, variation) * 0.006 * amount;
    pose.sideSway += Wave(clock, 1.0, variation * 3.0) * 0.02 * amount;
    pose.leftArmRoll += 0.03 * amount;
    pose.rightArmRoll -= 0.03 * amount;
}

// Glancing from side to side, pausing in between, with the shoulders following a little.
void AddLookAround(inout Pose pose, float clock, float variation, float weight) {
    float glance = Wave(clock, 2.0, variation) * 0.7 + Wave(clock, 3.0, variation * 5.0) * 0.3;
    pose.headTurn += glance * 0.85 * weight;
    pose.bodyTwist += glance * 0.2 * weight;
    pose.headNod += (Wave(clock, 1.0, variation * 2.0) * 0.5 - 0.1) * 0.15 * weight;
}

// Talking with the hands: the right forearm comes up and beats with the speech, the left
// joins in now and then, and the head nods. Listeners get the same at a lower weight.
void AddTalk(inout Pose pose, float clock, float variation, float weight) {
    float beat = Wave(clock, 5.0, variation) * 0.5 + 0.5;
    float emphasis = Wave(clock, 2.0, variation * 3.0) * 0.5 + 0.5;
    pose.rightArmPitch -= (0.55 + 0.45 * beat * emphasis) * weight;
    pose.rightArmRoll -= 0.15 * emphasis * weight;
    pose.leftArmPitch -= (0.2 + 0.5 * max(0.0, Wave(clock, 3.0, variation * 7.0))) * weight;
    pose.leftArmRoll += 0.1 * weight;
    pose.headNod += Wave(clock, 7.0, variation) * 0.08 * weight;
    pose.headTurn += Wave(clock, 1.0, variation * 4.0) * 0.15 * weight;
    pose.bodyTwist += Wave(clock, 1.0, variation * 4.0 + 0.25) * 0.06 * weight;
}

Pose PoseOfInstance() {
    float phase = inAnimation.x;
    float stride = inAnimation.y;
    int gestureKind = int(floor(inAnimation.z));
    float variation = fract(inAnimation.z);
    float gestureWeight = floor(inAnimation.w) / 64.0;
    float clock = fract(inAnimation.w);

    Pose pose = WalkPose(phase, stride, variation);
    float standing = clamp(1.0 - stride * 2.0, 0.0, 1.0);
    AddIdle(pose, clock, variation, standing);
    if (gestureKind == GESTURE_LOOK_AROUND) {
        AddLookAround(pose, clock, variation, gestureWeight);
    } else if (gestureKind == GESTURE_TALK) {
        // Hands are busy with walking, so talking only shows fully when standing.
        AddTalk(pose, clock, variation, gestureWeight * mix(0.3, 1.0, standing));
    }
    return pose;
}

// Moves a point (or, with `pivot` zero and no translation, a normal) of a limb around its
// joint. Rotations are applied innermost first.
vec3 RotateLimb(vec3 v, vec3 pivot, float pitch, float roll) {
    return pivot + RotateAroundX(RotateAroundZ(v - pivot, roll), pitch);
}

// The part of the pose that depends on the body part: everything from the limb out to the
// hips. Returns the moved position and turns `normal` the same way.
vec3 PoseBodyPart(int part, vec3 position, inout vec3 normal, Pose pose) {
    bool isLeg = part == PART_LEFT_LEG || part == PART_RIGHT_LEG;
    if (isLeg) {
        bool isLeft = part == PART_LEFT_LEG;
        // The lower end of the leg bends back around the knee; the leg box has corners only
        // at the hip and the foot, so this bends the foot end of the box.
        if (position.y < KNEE_HEIGHT) {
            float knee = isLeft ? pose.leftKnee : pose.rightKnee;
            vec3 kneeJoint = vec3(position.x, KNEE_HEIGHT, 0.0);
            position = kneeJoint + RotateAroundX(position - kneeJoint, knee);
        }
        float hipAngle = isLeft ? pose.legSwing : -pose.legSwing;
        normal = RotateAroundX(normal, hipAngle);
        return HIP + RotateAroundX(position - HIP, hipAngle);
    }

    if (part == PART_LEFT_ARM) {
        position = RotateLimb(position, LEFT_SHOULDER, pose.leftArmPitch, pose.leftArmRoll);
        normal = RotateAroundX(RotateAroundZ(normal, pose.leftArmRoll), pose.leftArmPitch);
    } else if (part == PART_RIGHT_ARM) {
        position = RotateLimb(position, RIGHT_SHOULDER, pose.rightArmPitch, pose.rightArmRoll);
        normal = RotateAroundX(RotateAroundZ(normal, pose.rightArmRoll), pose.rightArmPitch);
    } else if (part == PART_HEAD) {
        position = NECK + RotateAroundX(RotateAroundY(position - NECK, pose.headTurn), pose.headNod);
        normal = RotateAroundX(RotateAroundY(normal, pose.headTurn), pose.headNod);
    }
    // Everything above the legs breathes, leans and twists with the upper body.
    position.y += pose.breath * smoothstep(1.0, 1.4, position.y);
    position = HIP + RotateAroundX(RotateAroundY(position - HIP, pose.bodyTwist), pose.bodyLean);
    normal = RotateAroundX(RotateAroundY(normal, pose.bodyTwist), pose.bodyLean);
    return position;
}

void main() {
    int part = int(inPart + 0.5);
    Pose pose = PoseOfInstance();

    vec3 normal = inNormal;
    vec3 position = PoseBodyPart(part, inPosition, normal, pose);
    position += vec3(pose.sideSway, pose.bounce, 0.0);
    position *= inColorHeight.w;

    float heading = inPositionHeading.w;
    vec3 worldPosition = inPositionHeading.xyz + RotateAroundY(position, heading);

    // The skin tone comes from a hash of the clothing colour, so it stays the same for each person
    // even as people enter and leave the screen (which reorders the instances).
    vec3 clothing = inColorHeight.rgb;
    float skinTone = Hash(dot(clothing, vec3(12.9898, 78.233, 37.719)));
    vec3 skin = mix(vec3(0.80, 0.60, 0.46), vec3(0.36, 0.24, 0.17), skinTone);
    vec3 albedo = clothing;
    if (part == PART_HEAD) {
        albedo = skin;
    } else if (part == PART_LEFT_LEG || part == PART_RIGHT_LEG) {
        albedo = clothing * 0.35 + vec3(0.02);  // darker trousers or skirts
    }

    outWorldPosition = worldPosition;
    outNormal = RotateAroundY(normal, heading);
    outLocalNormal = outNormal;
    outAlbedo = albedo;
    outSurface = vec3(0.75, MATERIAL_CHARACTER, skinTone);
    outLocalPosition = position;
    outBoxSize = vec3(1.0);
    gl_Position = push.viewProjection * vec4(worldPosition, 1.0);
}
