/*
 * Copyright 2022 Leon.
 */
package com.chris.depression.data

import android.content.Context
import android.media.MediaCodec
import android.media.MediaExtractor
import android.media.MediaFormat
import android.media.MediaMuxer
import android.media.MediaMuxer.OutputFormat
import android.os.Build.VERSION
import android.os.Build.VERSION_CODES
import android.os.Environment
import timber.log.Timber
import java.io.File
import java.nio.ByteBuffer

object SplitMediaFile {

    private var pcmPath = ""
    private var mp4Path = ""

    fun extractAudio(context: Context, video: File): File {
        val dirPath = getSDPath(context)
        pcmPath = dirPath + "/${video.name}.aac"
        mp4Path = dirPath + "/${video.name}.h264"
        val mediaDir = File(dirPath)
        val audioFile = File(pcmPath)
//        val videoFile = File(mp4Path)
        kotlin.runCatching {
            if (!mediaDir.exists()) mediaDir.mkdir()
            if (audioFile.exists()) audioFile.delete()
            audioFile.createNewFile()
//            if (videoFile.exists()) videoFile.delete()
//            videoFile.createNewFile()
        }.onFailure {
            Timber.e(it)
        }
        audioFile.deleteOnExit()
        val mediaExtractor = MediaExtractor()
        mediaExtractor.setDataSource(video.absolutePath)
        val muxer = MediaMuxer(pcmPath, OutputFormat.MUXER_OUTPUT_MPEG_4)
        try {
            var audioTrackIndex = -1
            for (x in 0 until mediaExtractor.trackCount) {
                val format = mediaExtractor.getTrackFormat(x)
                //只需音轨
                if (format.getString(MediaFormat.KEY_MIME)?.startsWith("audio/") == true) {
                    audioTrackIndex = muxer.addTrack(format)
                    mediaExtractor.selectTrack(x)
                }
            }
            val bufferSize = 512 * 1024
            val offset = 0
            val dstBuf = ByteBuffer.allocate(bufferSize)
            val bufferInfo = MediaCodec.BufferInfo()
            muxer.start()
            while (true) {
                bufferInfo.offset = offset
                bufferInfo.size = mediaExtractor.readSampleData(dstBuf, offset)
                if (bufferInfo.size < 0) {
                    mediaExtractor.unselectTrack(audioTrackIndex)
                    break
                } else {
                    bufferInfo.presentationTimeUs = mediaExtractor.sampleTime
                    bufferInfo.flags = mediaExtractor.sampleFlags
                    muxer.writeSampleData(audioTrackIndex, dstBuf, bufferInfo)
                    mediaExtractor.advance()
                }
            }
            muxer.stop()
            muxer.release()
            mediaExtractor.release()
        } catch (e: Exception) {
            e.printStackTrace()
        } finally {
            try {
                muxer.stop()
            } catch (e: Exception) {
                e.printStackTrace()
            }
            muxer.release()
            mediaExtractor.release()
        }
        return audioFile
    }

    //    fun getAudioFromVideo(context: Context, video: File): File {
//        val mediaExtractor = MediaExtractor() // 此类可分离视频文件的音轨和视频轨道
//        val dirPath = getSDPath(context)
//        pcmPath = dirPath + "/${video.name}.aac"
//        mp4Path = dirPath + "/${video.name}.h264"
//        val mediaDir = File(dirPath)
//        val audioFile = File(pcmPath)
//        val videoFile = File(mp4Path)
//        kotlin.runCatching {
//            if (!mediaDir.exists()) mediaDir.mkdir()
//            if (audioFile.exists()) audioFile.delete()
//            audioFile.createNewFile()
//            if (videoFile.exists()) videoFile.delete()
//            videoFile.createNewFile()
//        }.onFailure {
//            Timber.e(it)
//        }
//        try {
//            mediaExtractor.setDataSource(video.path) // 媒体文件的位置
//            Timber.i("==========getTrackCount()===============${mediaExtractor.trackCount}")
//            for (i in 0 until mediaExtractor.trackCount) { // 遍历媒体轨道，包括视频和音频轨道
//                val format: MediaFormat = mediaExtractor.getTrackFormat(i)
//                val mime: String? = format.getString(MediaFormat.KEY_MIME)
//                if (mime?.startsWith("audio") == true) { // 获取音频轨道
//                    mediaExtractor.selectTrack(i) // 选择此音频轨道
//                    Timber.i("==audio==KEY_MIME===${format.getString(MediaFormat.KEY_MIME)}")
//                    Timber.i("==audio==KEY_CHANNEL_COUNT===${format.getString(MediaFormat.KEY_CHANNEL_COUNT)}")
//                    Timber.i("==audio==KEY_SAMPLE_RATE===${format.getString(MediaFormat.KEY_SAMPLE_RATE)}")
//                    Timber.i("==audio==KEY_DURATION===${format.getString(MediaFormat.KEY_DURATION)}")
//                    Timber.i("==audio==getSampleFlags===${mediaExtractor.sampleFlags}")
//                    Timber.i("==audio==getSampleTime===${mediaExtractor.sampleTime}")
//                    Timber.i("==audio==getSampleSize===${mediaExtractor.sampleSize}")
//                    Timber.i("==audio==getSampleTrackIndex===${mediaExtractor.sampleTrackIndex}")
//                    try {
//                        val inputBuffer: ByteBuffer = ByteBuffer.allocate(100 * 1024)
//                        val fe = FileOutputStream(audioFile, true)
//                        while (true) {
//                            val readSampleCount = mediaExtractor.readSampleData(inputBuffer, 0)
//                            if (readSampleCount <= 0) {
//                                break
//                            }
//                            val buffer = ByteArray(readSampleCount)
//                            inputBuffer.get(buffer)
//                            fe.write(buffer)
//                            inputBuffer.clear()
//                            mediaExtractor.advance()
//                        }
//                        fe.flush()
//                        fe.close()
//                    } catch (e: IOException) {
//                        e.printStackTrace()
//                    } finally {
//                    }
//                }
////                if (mime?.startsWith("video") == true) {
////                    mediaExtractor.selectTrack(i) //选择此视频轨道
////                    Timber.i("==video==KEY_MIME===" + format.getString(MediaFormat.KEY_MIME))
////                    Timber.i("==video==KEY_DURATION===" + format.getLong(MediaFormat.KEY_DURATION))
////                    Timber.i("==video==KEY_WIDTH===" + format.getInteger(MediaFormat.KEY_WIDTH))
////                    Timber.i("==video==KEY_HEIGHT===" + format.getInteger(MediaFormat.KEY_HEIGHT))
////                    Timber.i("==video==getSampleFlags===" + mediaExtractor.sampleFlags)
////                    Timber.i("==video==getSampleTime===" + mediaExtractor.sampleTime)
////                    Timber.i("==video==getSampleSize===" + mediaExtractor.sampleSize)
////                    Timber.i("==video==getSampleTrackIndex===" + mediaExtractor.sampleTrackIndex)
////                    try {
////                        val inputBuffer: ByteBuffer = ByteBuffer.allocate(100 * 1024)
////                        val fe = FileOutputStream(videoFile, true)
////                        while (true) {
////                            val readSampleCount = mediaExtractor.readSampleData(inputBuffer, 0)
////                            if (readSampleCount <= 0) {
////                                break
////                            }
////                            val buffer = ByteArray(readSampleCount)
////                            inputBuffer.get(buffer)
////                            fe.write(buffer)
////                            inputBuffer.clear()
////                            mediaExtractor.advance()
////                        }
////                        fe.flush()
////                        fe.close()
////                    } catch (e: IOException) {
////                        e.printStackTrace()
////                    } finally {
////                    }
////                }
//            }
//        } catch (e: IOException) {
//            e.printStackTrace()
//        } finally {
//            mediaExtractor.release()
//        }
//        return audioFile
//    }

    private fun getSDPath(context: Context): String {
        return if (Environment.getExternalStorageState() == Environment.MEDIA_MOUNTED) {
            if (VERSION.SDK_INT >= VERSION_CODES.Q) {
                context.getExternalFilesDir(null)
            } else {
                Environment.getExternalStorageDirectory() // 获取SD卡根目录
            }
        } else {
            Environment.getRootDirectory() // 获取跟目录
        }.toString()
    }
}
