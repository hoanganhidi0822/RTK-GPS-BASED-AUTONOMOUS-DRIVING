from transformers import SegformerForSemanticSegmentation, SegformerConfig

def segformer_model(classes):
    id2label = {i: name for i, name in enumerate(classes)}
    label2id = {name: i for i, name in enumerate(classes)}

    config = SegformerConfig.from_pretrained(
        'nvidia/segformer-b0-finetuned-cityscapes-768-768',
        num_labels=len(classes),
        id2label=id2label,
        label2id=label2id,
        ignore_mismatched_sizes=True
    )

    model = SegformerForSemanticSegmentation.from_pretrained(
        'nvidia/segformer-b0-finetuned-cityscapes-768-768',
        config=config,
        ignore_mismatched_sizes=True
    )
    return model
