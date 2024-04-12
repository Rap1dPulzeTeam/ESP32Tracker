def calculate_frequency(finetune_value, base_frequency=8287):
    # 基本微调步进值
    semitone_step = 1 / 8

    # 根据微调值计算微调量
    if finetune_value == 0x00:
        fine_adjustment = 0
    elif finetune_value <= 0x07:
        fine_adjustment = semitone_step * finetune_value
    else:
        fine_adjustment = -semitone_step * (0x10 - finetune_value)

    # 计算调整后的频率
    adjusted_frequency = base_frequency * (2 ** (fine_adjustment / 12))

    # 返回频率 * 428
    return adjusted_frequency * 428

