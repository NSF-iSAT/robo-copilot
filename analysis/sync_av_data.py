# import ffmpeg
from os import system

def get_sync_pts(vid_filename, audio_filename):
    vid_timestamp = input("Enter the video timestamp for {} in m:ss format: ".format(vid_filename))
    vid_timestamp = vid_timestamp.split(":")
    vid_timestamp = int(vid_timestamp[0]) * 60 + int(vid_timestamp[1])

    av_timestamp = input("Enter the audiovisual timestamp for {} in m:ss format: ".format(audio_filename))
    av_timestamp = av_timestamp.split(":")
    av_timestamp = int(av_timestamp[0]) * 60 + int(av_timestamp[1])

    return vid_timestamp, av_timestamp

def get_trim_pts(audio_filename):
    trim_pt = input("Enter the timestamp for {} to start at, in m:ss format: ".format(audio_filename))
    trim_pt = trim_pt.split(":")
    trim_pt = int(trim_pt[0]) * 60 + int(trim_pt[1])

    end_pt = input("Enter the timestamp for {} to end at, in m:ss format: ".format(audio_filename))
    end_pt = end_pt.split(":")
    end_pt = int(end_pt[0]) * 60 + int(end_pt[1]) - trim_pt

    return trim_pt, end_pt

def update_vtt(vtt_filename, start_time, end_duration):
    output = []
    in_bounds = False
    with open(vtt_filename, "r") as vtt_file:
        for line in vtt_file:
            if "-->" in line:
                utterance_start = line.split("-->")[0].strip()
                utterance_start = utterance_start.split(":")
                utterance_start = int(utterance_start[0]) * 3600 + int(utterance_start[1]) * 60 + float(utterance_start[2])

                utterance_end = line.split("-->")[1].strip()
                utterance_end = utterance_end.split(":")
                utterance_end = int(utterance_end[0]) * 3600 + int(utterance_end[1]) * 60 + float(utterance_end[2])

                if utterance_start >= start_time:
                    in_bounds = True
                elif utterance_start > start_time + end_duration:
                    in_bounds = False
                    break
                else:
                    continue

                new_timestamp_start = utterance_start - start_time
                new_timestamp_end   = utterance_end  - start_time

                new_timestamp_start = "%02d:%02d:%02d" % (new_timestamp_start // 3600, (new_timestamp_start % 3600) // 60, new_timestamp_start % 60)
                new_timestamp_end   = "%02d:%02d:%02d" % (new_timestamp_end // 3600, (new_timestamp_end % 3600) // 60, new_timestamp_end % 60)

                line = "%s --> %s\n" % (new_timestamp_start, new_timestamp_end)
                output.append(line)

            elif in_bounds:
                output.append(line)

    output_file = vtt_filename.split(".")[0] + "_trimmed.vtt"
    with open(output_file, "w") as vtt_file:
        vtt_file.writelines(output)


def sync_av_data(mkv_file, mp4_file_a, vtt_file):
    # vid_timestamp, av_timestamp = get_sync_pts(mkv_file, mp4_file_a)
    start_pt, end_pt = get_trim_pts(mp4_file_a)

    # time_offset = vid_timestamp - av_timestamp

    # trimmed_mp4 = mp4_file_a.split(".")[0] + "_trimmed.mp4"
    # trim_file(mp4_file_a, start_pt, end_pt, trimmed_mp4)

    # trimmed_mp4_audio_only = trimmed_mp4.split(".")[0] + "_trimmed_audio_only.mp4"
    # trim_file(mp4_file_b, start_pt, end_pt, trimmed_mp4_audio_only)

    # trimmed_obs_mkv = mkv_file.split(".")[0] + "_trimmed.mkv"
    # trim_file(mkv_file, start_pt + time_offset, end_pt, trimmed_obs_mkv)

    update_vtt(vtt_file, start_pt, end_pt)

def trim_file(filename, start_pt, end_pt, outfile):
    system("ffmpeg -i %s -ss %s -to %s -c:v copy -c:a copy %s" % (filename, start_pt, end_pt, outfile))

if __name__ == "__main__":
    mp4_file = "zoom.mp4"
    mkv_file = "obs.mkv"
    vtt_file = "zoom.vtt"
    sync_av_data(mkv_file, mp4_file, vtt_file)
    
