//
// Created by Loboda Alexey on 21.05.2020.
//

#include <string>
#include <jni.h>
#include "codec/CodecOpus.h"
#include "utils/SamplesConverter.h"

CodecOpus codec;

//
// Encoding
//

extern "C"
JNIEXPORT jint JNICALL Java_com_theeasiestway_opus_Opus_encoderInit(JNIEnv *env, jobject thiz, jint sample_rate, jint num_channels, jint application) {
    return codec.encoderInit(sample_rate, num_channels, application);
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_theeasiestway_opus_Opus_encoderSetBitrate(JNIEnv *env, jobject thiz, jint bitrate) {
    return codec.encoderSetBitrate(bitrate);
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_theeasiestway_opus_Opus_encoderSetComplexity(JNIEnv *env, jobject thiz, jint complexity) {
    return codec.encoderSetComplexity(complexity);
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_theeasiestway_opus_Opus_encode___3BI(JNIEnv *env, jobject thiz, jbyteArray bytes, jint frame_size) {
    void *nativeBytes = env->GetPrimitiveArrayCritical(bytes, nullptr);
    std::vector<uint8_t> encodedData = codec.encode((uint8_t *) nativeBytes, frame_size);
    env->ReleasePrimitiveArrayCritical(bytes, nativeBytes, JNI_ABORT);

    int encodedSize = encodedData.size();
    if (encodedSize <= 0) return nullptr;

    jbyteArray result = env->NewByteArray(encodedSize);
    env->SetByteArrayRegion(result, 0, encodedSize, (jbyte *) encodedData.data());

    return result;
}

extern "C"
JNIEXPORT jshortArray JNICALL
Java_com_theeasiestway_opus_Opus_encode___3SI(JNIEnv *env, jobject thiz, jshortArray shorts, jint frame_size) {
    jint length = env->GetArrayLength(shorts);

    void *nativeShorts = env->GetPrimitiveArrayCritical(shorts, nullptr);
    std::vector<short> encodedData = codec.encode((short *) nativeShorts, length, frame_size);
    env->ReleasePrimitiveArrayCritical(shorts, nativeShorts, JNI_ABORT);

    int encodedSize = encodedData.size();
    if (encodedSize <= 0) return nullptr;

    jshortArray result = env->NewShortArray(encodedSize);
    env->SetShortArrayRegion(result, 0, encodedSize, encodedData.data());

    return result;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_theeasiestway_opus_Opus_encoderRelease(JNIEnv *env, jobject thiz) {
    codec.encoderRelease();
}

//
// Decoding
//

extern "C"
JNIEXPORT jint JNICALL
Java_com_theeasiestway_opus_Opus_decoderInit(JNIEnv *env, jobject thiz, jint sample_rate, jint num_channels) {
    return codec.decoderInit(sample_rate, num_channels);
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_theeasiestway_opus_Opus_decode___3BI(JNIEnv *env, jobject thiz, jbyteArray bytes, jint frame_size) {
    jint length = env->GetArrayLength(bytes);

    void *nativeBytes = env->GetPrimitiveArrayCritical(bytes, nullptr);
    std::vector<uint8_t> encodedData = codec.decode((uint8_t *) nativeBytes, length, frame_size);
    env->ReleasePrimitiveArrayCritical(bytes, nativeBytes, JNI_ABORT);

    int encodedSize = encodedData.size();
    if (encodedSize <= 0) return nullptr;

    jbyteArray result = env->NewByteArray(encodedSize);
    env->SetByteArrayRegion(result, 0, encodedSize, (jbyte *) encodedData.data());

    return result;
}

extern "C"
JNIEXPORT jshortArray JNICALL
Java_com_theeasiestway_opus_Opus_decode___3SI(JNIEnv *env, jobject thiz, jshortArray shorts, jint frame_size) {
    jint length = env->GetArrayLength(shorts);

    void *nativeShorts = env->GetPrimitiveArrayCritical(shorts, nullptr);
    std::vector<short> encodedData = codec.decode((short *) nativeShorts, length, frame_size);
    env->ReleasePrimitiveArrayCritical(shorts, nativeShorts, JNI_ABORT);

    int encodedSize = encodedData.size();
    if (encodedSize <= 0) return nullptr;

    jshortArray result = env->NewShortArray(encodedSize);
    env->SetShortArrayRegion(result, 0, encodedSize, encodedData.data());

    return result;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_theeasiestway_opus_Opus_decoderRelease(JNIEnv *env, jobject thiz) {
    codec.decoderRelease();
}

//
// Utils
//

extern "C"
JNIEXPORT jshortArray JNICALL
Java_com_theeasiestway_opus_Opus_convert___3B(JNIEnv *env, jobject thiz, jbyteArray bytes) {
    jint length = env->GetArrayLength(bytes);

    void *nativeBytes = (uint8_t *) env->GetPrimitiveArrayCritical(bytes, nullptr);
    std::vector<short> shorts = SamplesConverter::convert((uint8_t **) &nativeBytes, length);
    env->ReleasePrimitiveArrayCritical(bytes, nativeBytes, JNI_ABORT);

    int size = shorts.size();
    if (!size) return nullptr;

    jshortArray result = env->NewShortArray(size);
    env->SetShortArrayRegion(result, 0, size, shorts.data());

    return result;
}

extern "C"
JNIEXPORT jbyteArray JNICALL
Java_com_theeasiestway_opus_Opus_convert___3S(JNIEnv *env, jobject thiz, jshortArray shorts) {
    jint length = env->GetArrayLength(shorts);

    void *nativeShorts = env->GetPrimitiveArrayCritical(shorts, nullptr);
    std::vector<uint8_t> bytes = SamplesConverter::convert((short **) &nativeShorts, length);
    env->ReleasePrimitiveArrayCritical(shorts, nativeShorts, JNI_ABORT);

    int size = bytes.size();
    if (!size) return nullptr;

    jbyteArray result = env->NewByteArray(size);
    env->SetByteArrayRegion(result, 0, size, (jbyte *) bytes.data());

    return result;
}