# %%
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import datetime
import os
import torch
from torch import nn
from torch.nn.modules import conv
from torch.nn.modules.pooling import AdaptiveAvgPool1d
from torchvision import datasets, transforms, models
from torch.utils.data import DataLoader, Dataset, random_split
import torch.nn.functional as F

root_dir = './'
models_path = './models'
lookup_filename = 'vr_handheld_controller_gesture_data.csv'
lookup_filepath = os.path.join(root_dir, lookup_filename)
data_path = './data_files'
dtype = torch.float32
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# %%
def read_and_interpolate_features(filepath, signal_length=150):
    features = torch.tensor(pd.read_csv(filepath, header=None, dtype=float, na_values=['ovf']).dropna(axis=1).to_numpy())
    features_idx_interp = F.interpolate(torch.unsqueeze(features, dim=0), size=signal_length)
    return torch.squeeze(features_idx_interp, dim=0)

# %%
class GestureDataset(Dataset):
    def __init__(self, lookup_filepath, signal_set_length=150):
        self.lookup_csv = pd.read_csv(lookup_filepath, header=None)
        self.signal_set_length = signal_set_length
        
    def __len__(self):
        return self.lookup_csv.shape[0]

    def __getitem__(self, idx):
        data_idx = self.lookup_csv.iloc[idx,:]
        filepath_idx = data_idx[0]
        label_idx = torch.tensor(data_idx[1] - 1, dtype=torch.long) # Offset labels by 1 so they are zero-based, note label 1 will correspond to label 0 now etc.
        dt_idx = data_idx[2]
        features_idx_interp_squeezed = read_and_interpolate_features(filepath_idx, signal_length=self.signal_set_length)
        return features_idx_interp_squeezed, label_idx


# %%
class GestureClassificationNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
            nn.Conv1d(9,16,kernel_size=3),
            nn.ReLU(),
            nn.BatchNorm1d(16),
            nn.AdaptiveAvgPool1d(125)
        )
        self.conv2 = nn.Sequential(
            nn.Conv1d(16,32,kernel_size=5),
            nn.ReLU(),
            nn.BatchNorm1d(32),
            nn.AdaptiveAvgPool1d(12)
        )
        self.fc = nn.Sequential(
            nn.Flatten(start_dim=1, end_dim=2),
            nn.Linear(12*32,6),
            nn.Dropout(p=0.5)
        ) 
        self.test = nn.Sequential(
            nn.Linear(150,30),
            nn.ReLU(),
            nn.BatchNorm1d(9),
            nn.Dropout(),
            nn.Flatten(start_dim=1),
            nn.Linear(30*9,6)
        )

    def forward(self, signals):
        conv1_out = self.conv1(signals)
        conv2_out = self.conv2(conv1_out)
        score = self.fc(conv2_out)
        # score = self.test(signals)
        return score

# %%
def check_error(model, loader, device, dtype):
    model.eval()
    model = model.to(device=device, dtype=dtype)
    total_num_vals = 0
    total_num_incorrect = 0
    with torch.no_grad():
        for batch in loader:
            features, labels = batch
            features = features.to(device=device, dtype=dtype)
            labels = labels.to(device=device, dtype=torch.long)
            scores = model(features)
            prediction = torch.argmax(scores, dim=1)
            num_incorrect = torch.sum(prediction != labels)
            total_num_incorrect += num_incorrect
            total_num_vals += scores.shape[0]
    
    return (total_num_incorrect/total_num_vals)


# %%
if __name__=='__main__':
    # Load dataset 
    full_datset = GestureDataset(lookup_filepath)

    # %%
    # Split into train and test
    test_prop = 0.2
    test_size = int(test_prop*len(full_datset))
    train_size = len(full_datset) - test_size
    train_dataset, test_dataset = random_split(full_datset, (train_size, test_size)) 

    # Split train into train and val
    val_prop = 0.2
    val_size = int(val_prop*len(train_dataset))
    train_size = len(train_dataset) - val_size
    train_dataset, val_dataset = random_split(train_dataset, (train_size, val_size)) 
    # %%
    batch_size = 16 
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset)
    test_loader = DataLoader(test_dataset)
    # %%
    # Visualize batch sizes 
    for batch_idx, batch in enumerate(train_loader):
        features, labels = batch
        print(features.shape)

    # %%
    # Visualize batch
    random_batch_idx = random.randint(0, len(train_loader)-1)
    for batch_idx, batch in enumerate(train_loader):
        if batch_idx != random_batch_idx:
            continue
        features, labels = batch
        for example_in_batch_idx, feature in enumerate(features):
            label = labels[example_in_batch_idx]
            plt.figure()
            for signal in feature:
                plt.title('Signal Plot for Label = {0} (zero-based label of {1})'.format(label+1, label))
                plt.plot(signal)
            plt.show()
    # %%
    model = GestureClassificationNet()
    learning_rate = 5e-3
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate, weight_decay=1e-7)
    model = model.to(device=device, dtype=dtype)
    nepochs = 20
    epoch_train_loss_list = []
    epoch_train_err_list = []
    epoch_val_err_list = []
    best_val_err = 1.0
    date_str = datetime.datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
    model_save_name = 'gesture_model_{0}.pt'.format(date_str)
    model_save_path = os.path.join(models_path, model_save_name)
    for epoch in range(nepochs):
        model.train()
        print('Beginning epoch {0}'.format(epoch))
        for batch in train_loader:
            features, labels = batch
            features = features.to(device=device, dtype=dtype)
            labels = labels.to(device=device, dtype=torch.long)
            score = model(features)
            loss = F.cross_entropy(score, labels)

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        epoch_train_loss_list.append(loss.detach().cpu())
        train_err = check_error(model, train_loader, device=device, dtype=dtype)
        val_err = check_error(model, val_loader, device=device, dtype=dtype)
        epoch_train_err_list.append(train_err)
        epoch_val_err_list.append(val_err)
        print('Train error = {0}, val error = {1}\n\n'.format(train_err, val_err))

        if val_err < best_val_err:
            torch.save(model.state_dict(), model_save_path)
            best_val_err = val_err
    print('Best model in terms of val error saved as {0}'.format(model_save_path))
    # %%
    plt.figure()
    plt.plot(epoch_train_loss_list, label='Train Loss')
    plt.title('Loss vs Epoch')
    plt.xlabel('Epoch')
    plt.ylabel('Cross Entropy Loss')
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(epoch_train_err_list, label='Train Error')
    plt.plot(epoch_val_err_list, label='Val Error')
    plt.title('Error vs Epoch')
    plt.xlabel('Epoch')
    plt.ylabel('Error')
    plt.legend()
    plt.show()

    # %%
    model_save_path = model_save_path
    # Load best model in terms of val error
    best_model = GestureClassificationNet()
    best_model.load_state_dict(torch.load(model_save_path))
    train_err = check_error(best_model, train_loader, device=device, dtype=dtype)
    val_err = check_error(best_model, val_loader, device=device, dtype=dtype)
    test_err = check_error(best_model, test_loader, device=device, dtype=dtype)
    print('Best model results: train error = {0}, val error = {1}, test error = {2}\n\n'.format(train_err, val_err, test_err))
