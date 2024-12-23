from torch.utils.data import DataLoader

from datasets import SemanticKitti, collate_fn


def get_dataset(_cfg):
    
    data_root =" /root/HE-Nav/src/perception/raw_data/voxels"
    config_file = "/root/HE-Nav/src/perception/LBSCNet/data/semantic-kitti.yaml"
    lims = _cfg['DATASET']['LIMS']
    sizes = _cfg['DATASET']['SIZES']
    #ds_train = SemanticKitti(data_root, config_file, 'train', lims, sizes, augmentation=True, shuffle_index=True)
    #ds_val = SemanticKitti(data_root, config_file, 'valid', lims, sizes, augmentation=False, shuffle_index=False)
    ds_test = SemanticKitti(data_root, config_file, 'test', lims, sizes, augmentation=False, shuffle_index=False)
    dataset = {}
    train_batch_size = _cfg['TRAIN']['BATCH_SIZE']
    val_batch_size = _cfg['VAL']['BATCH_SIZE']
    num_workers = 0
 
    #dataset['train'] = DataLoader(ds_train, batch_size=train_batch_size, num_workers=num_workers, shuffle=True, collate_fn=collate_fn)
    #dataset['val'] = DataLoader(ds_val, batch_size=val_batch_size, num_workers=num_workers, shuffle=False, collate_fn=collate_fn)
    dataset['test'] = DataLoader(ds_test, batch_size=train_batch_size, num_workers=num_workers, shuffle=False, collate_fn=collate_fn)

    return dataset