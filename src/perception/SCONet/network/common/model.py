from network.models.LMSCNet import LMSCNet
from network.models.LMSCNet_SS import LMSCNet_SS
from network.models.SSCNet_full import SSCNet_full
from network.models.SSCNet import SSCNet
from network.models.SCONet import SCONet



def get_model(_cfg, dataset):

  nbr_classes = dataset.nbr_classes
  grid_dimensions = dataset.grid_dimensions
  class_frequencies = dataset.class_frequencies

  selected_model = _cfg._dict['MODEL']['TYPE']
  
  # SCONet -------------------------------------------------------------------------------------------------------
  if selected_model == 'SCONet':
    model = SCONet(class_num=nbr_classes, input_dimensions=grid_dimensions, class_frequencies=class_frequencies)
  # ------------------------------------------------------------------------------------------------------------------

  # LMSCNet ----------------------------------------------------------------------------------------------------------
  elif selected_model == 'LMSCNet':
    model = LMSCNet(class_num=nbr_classes, input_dimensions=grid_dimensions, class_frequencies=class_frequencies)
  # ------------------------------------------------------------------------------------------------------------------

  # LMSCNet_SS -------------------------------------------------------------------------------------------------------
  elif selected_model == 'LMSCNet_SS':
    model = LMSCNet_SS(class_num=nbr_classes, input_dimensions=grid_dimensions, class_frequencies=class_frequencies)
  # ------------------------------------------------------------------------------------------------------------------
  
  # SSCNet_full ------------------------------------------------------------------------------------------------------
  elif selected_model == 'SSCNet_full':
    model = SSCNet_full(class_num=nbr_classes)
  # ------------------------------------------------------------------------------------------------------------------

  # SSCNet -----------------------------------------------------------------------------------------------------------
  elif selected_model == 'SSCNet':
    model = SSCNet(class_num=nbr_classes)
  # ------------------------------------------------------------------------------------------------------------------

  else:
    assert False, 'Wrong model selected'

  return model