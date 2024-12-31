#!/usr/bin/python3
# Created 2024-12-07
# TJ Wiegman
# for ASM 591 AI final project

# Files to work with
training_videos = [
    '/mnt/nas/2024/Tractor02/2024-06-18_083656.MP4',
    '/mnt/nas/2024/Tractor02/2024-06-18_120818.MP4',
    # '/mnt/nas/2024/Tractor02/2024-06-19_083249.MP4',
    # '/mnt/nas/2024/Tractor02/2024-06-19_095128.MP4'
]
testing_videos = [
    # '/mnt/nas/2024/Tractor02/2024-06-18_083656.MP4',
    # '/mnt/nas/2024/Tractor02/2024-06-18_120818.MP4',
    '/mnt/nas/2024/Tractor02/2024-06-19_083249.MP4',
    # '/mnt/nas/2024/Tractor02/2024-06-19_095128.MP4'
]
model_checkpoint = "asm591ai.tar"

# Training/Testing controls
import os
MAX_EPOCHS = 1
checkpoint_already_exists = os.path.exists(model_checkpoint)

# Enable Pytorch GPU acceleration
import torch
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(device)

# NN Model
import torch.nn as nn
import torch.nn.functional as F
BATCH = 20

class VisOdoNet(nn.Module):
    def __init__(self):
        super().__init__()
        # Convolutional layers to learn spatial features
        self.conv1 = nn.Conv2d(4, 5, kernel_size=5, padding=2)
        self.conv2 = nn.Conv2d(5, 3, kernel_size=3, padding=3, dilation=2)
        self.conv3 = nn.Conv2d(3, 1, kernel_size=5, padding=2)
        
        # LSTM network to learn temporal features
        self.rnn1 = nn.LSTM(input_size=256,
                            hidden_size=8, num_layers=4,
                            bidirectional=False, # should be monotonic
                            batch_first=True)
        
        # And a few FC layers to tie it all together
        self.fc1 = nn.Linear(64, 16)
        self.fc2 = nn.Linear(16, BATCH)
        
    def forward(self, x):         # input [B x 4 x 256 x 256]
        x = F.relu(self.conv1(x)) # shape [B x 5 x 256 x 256]
        x = F.avg_pool2d(x, 2)    # shape [B x 5 x 128 x 128]
        x = F.relu(self.conv2(x)) # shape [B x 3 x 128 x 128]
        x = F.max_pool2d(x, 4)    # shape [B x 3 x  32 x  32]
        x = F.relu(self.conv3(x)) # shape [B x 1 x  32 x  32]
        x = F.avg_pool2d(x, 2)    # shape [B x 1 x  16 x  16]
        x = x.reshape(-1, 256)    # shape [B x 256]
        
        _, (x, c) = self.rnn1(x)  # pull both hidden and cell states
        x = x.reshape(-1, 32) # B x 32 hidden state
        c = c.reshape(-1, 32) # B x 32 cell state
        x = torch.concat((x,c))
        x = F.relu(x.reshape(-1, 64))   # [B x 64]
        x = F.relu(self.fc1(x))   # shape [B x 16]
        x = F.relu(self.fc2(x))   # shape [BATCH]
        return x.reshape(-1)

# Utility Functions
import datetime
from dateutil import parser

def stamp(dt):
    '''Converts a datetime into a string suitable for indexing'''
    return f"{dt.date()}_{dt.hour:02}:{dt.minute:02}"

def unstamp(st):
    '''Converts a `stamp` string back into a datetime'''
    date, time = st.split("_")
    yy,mm,dd = date.split("-")
    hh,mn = time.split(":")
    return datetime.datetime(
        year=int(yy), month=int(mm), day=int(dd),
        hour=int(hh), minute=int(mn)
    )

def roll_avg(frame, new):
    N = frame[1] + 1
    output = frame[0]*(1 - 1/N) + new/N
    return output, N

# Import depth feature extraction model
import sys
sys.path.append("./")
from Depth_Anything_V2.depth_anything_v2.dpt import DepthAnythingV2
# From https://github.com/DepthAnything/Depth-Anything-V2#use-our-models
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}
encoder = 'vitb' # or 'vits', 'vitl', 'vitg'
depth_model = DepthAnythingV2(**model_configs[encoder])
depth_model.load_state_dict(torch.load(f"Depth_Anything_V2/checkpoints/depth_anything_v2_{encoder}.pth", weights_only=True, map_location='cpu'))
depth_model = depth_model.to(device).eval()


# Data structure for multithreaded video file streaming
import cv2, time
from threading import Thread
from queue import Queue

class FileVideoStream:
    # queue up frames in the background for faster playback
    # adapted from https://pyimagesearch.com/2017/02/06/faster-video-file-fps-with-cv2-videocapture-and-opencv/
    def __init__(self, path, queueSize=60):
        self.stream = cv2.VideoCapture(path)
        self.stopped = False
        self.Q = Queue(maxsize=queueSize)
    
    def more(self): return self.Q.qsize() > 0
    def read(self): return self.Q.get()
    def stop(self): self.stopped = True

    def update(self):
        while True:
            if self.stopped: return
            if not self.Q.full():
                (grabbed,frame) = self.stream.read()
                if not grabbed:
                    self.stop()
                    return
                self.Q.put(frame)

    def start(self):
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        time.sleep(1.0) # give the queue a second to fill a bit
        return self

# Create frame-loading dataset
from torch.utils.data import Dataset, DataLoader
import numpy as np
import cv2, json

class VideoSet(Dataset):
    def __init__(self, video_paths):
        self.jsons = []
        self.sources = []
        self.n_frames = 0
        self.videos = {}
        
        for path in video_paths:
            # Get metadata from JSON
            name, ext = path.split(".")
            assert ext.upper() == "MP4"
            jfile = name + ".json"
            with open(jfile) as file:
                jdata = json.load(file)
            
            # Calculate frame numbers
            startFrame = self.n_frames
            self.n_frames += jdata[-1]["frame"] + 1 # because zero indexed
            endFrame = self.n_frames
            
            # Save data to self
            self.sources.append((startFrame, endFrame, path))
            self.jsons.append(jdata)
            
    def source_lookup(self, idx):
        i = 0
        for start, end, _ in self.sources:
            if idx >= start and idx < end:
                return i
            i += 1
        raise IndexError(f"Index {idx} not found in {self.sources}")
    
    def load_video(self, path):
        self.videos[path] = FileVideoStream(path).start()
    
    def __len__(self):
        return self.n_frames
    
    def __getitem__(self, idx):
        # Lookup correct source for idx
        i = self.source_lookup(idx)
        start, _, path = self.sources[i]
        fidx = idx - start
        if path not in self.videos:
            self.load_video(path) # lazy loading
        
        # Get visual data
        frame = self.videos[path].read() # shape H x W x 3
        depth = np.expand_dims(depth_model.infer_image(frame), -1) # shape H x W x 1
        frame = np.concat([x for x in [frame, depth]], axis = 2) # shape H x W x 4
        frame = cv2.resize(src=frame, dsize=(256,256), interpolation=cv2.INTER_AREA)
        frame = torch.tensor(frame.transpose(2,0,1), # because cv2 and pytorch don't agree on axis order
                             dtype=torch.float) # shape 4 x H x W
        
        # Get timestamp
        time = stamp(parser.isoparse(self.jsons[i][fidx]["gps_time"]))
        return frame, time

# Import ground-truth ground-speed data
with open("gt_data.json", "r") as file:
    gt_speed = json.load(file)

# Create data loaders
train_set = VideoSet(training_videos)
test_set = VideoSet(testing_videos)

train_loader = DataLoader(
    dataset = train_set,
    batch_size = BATCH,
    shuffle = False # loads videos sequentially == faster reads
)
test_loader = DataLoader(
    dataset = test_set,
    batch_size = BATCH,
    shuffle = False
)

# Create training function
def train(epoch, model, device, optimizer, data_loader, loss_function, gt_data):
    try:
        # Prepare model
        model = model.to(device)
        model = model.train()
        for batch_idx, (frame, time) in enumerate(data_loader):
            optimizer.zero_grad()
            frame = frame.to(device)

            # Get ground truth for comparison
            preds = []
            for st in time:
                if st in gt_data:
                    preds.append(gt_data[st][0])
                else:
                    preds.append(np.nan)
            y = np.array(preds).reshape(-1)
            y = torch.tensor(y, dtype=torch.float).to(device)

            # Calculate and record output & loss
            output = model(frame)

            # Ensure dimensions match:
            if output.shape == y.shape:
                loss = loss_function(output, y)
                loss.backward()
                optimizer.step()
            else:
                print(f"Dimension mismatch between prediction {output} and ground truth {y}")
                print(f" --> Skipping loss calculation for {time}")

            # Periodically report on training progress
            print(f"\rEpoch {epoch}: Training {batch_idx*BATCH}/{len(data_loader.dataset)} " + 
                  f"(Loss: {loss.item():02.4})", end=" "*10)

        print(f"\rEpoch {epoch}: Trained {len(data_loader.dataset)}/{len(data_loader.dataset)} " + 
                  f"(Loss: {loss.item():02.4})" + " "*10)
        return (loss, None)
    except KeyboardInterrupt:
        return (loss, KeyboardInterrupt)

# Create testing function
def test(epoch, model, device, data_loader, loss_function, gt_data):
    # Prepare model and data
    model = model.to(device)
    model = model.eval()
    test_loss = []
    map = []
    
    with torch.no_grad():
        for batch_idx, (frame, time) in enumerate(data_loader):
            # Load data into `device`
            frame = frame.to(device)
            
            # Get ground truth for comparison
            preds = []
            for st in time:
                if st in gt_data:
                    preds.append(gt_data[st][0])
                else:
                    preds.append(np.nan)
            y = np.array(preds).reshape(-1)
            y = torch.tensor(y, dtype=torch.float).to(device)
            
            # Calculate loss and accuracy
            output = model(frame)
            
            # Ensure dimensions match
            if output.shape == y.shape:
                test_loss.append(loss_function(output, y).item())
                map.append(torch.mean(torch.abs((output - y) / y)) * 100)
            else:
                print(f"Dimension mismatch between prediction {output} and ground truth {y}")
                print(f" --> Skipping loss calculation for {time}")
            
            # Periodically report on testing progress
            print(f"\rEpoch {epoch}: Testing {batch_idx*BATCH}/{len(data_loader.dataset)}, estimated MAPE {torch.mean(torch.tensor(map)):02.4}%", end=" "*10)
        print(f"\rEpoch {epoch}: Testing {len(data_loader.dataset)}/{len(data_loader.dataset)}" + " "*10)
    
    # Report results
    test_loss = torch.mean(torch.tensor(test_loss))
    accuracy = torch.tensor(map)
    print(f"Test Result, epoch {epoch}: Avg loss {test_loss:04.4}, MAPE {torch.mean(accuracy):02.4}%")
    
    return accuracy


# Training/Testing Loop!
model = VisOdoNet()
optimizer = torch.optim.Adam(model.parameters())

if checkpoint_already_exists:
    checkpoint = torch.load(model_checkpoint, weights_only=True)
    model.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    min_epoch = checkpoint['epoch']
    loss = checkpoint['loss']
    print(f"Loaded model from {model_checkpoint}, trained to epoch {min_epoch} with loss {loss.item():0.4f}")
else:
    min_epoch = 1

for epoch in range(min_epoch, MAX_EPOCHS+1):
    loss = train(
        epoch=epoch,
        model=model,
        device=device,
        optimizer=optimizer,
        data_loader=train_loader,
        loss_function=F.mse_loss,
        gt_data=gt_speed["Tractor01"]
    )
    
    torch.save(
        {
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'loss': loss[0],
        },
        model_checkpoint)
    print(f"Saved model to {model_checkpoint}")
    
    if loss[1] == KeyboardInterrupt:
        print(f"Interrupted! Encountered {loss[1]}")
        break
    
    accuracy = test(
        epoch=epoch,
        model=model,
        device=device,
        data_loader=test_loader,
        loss_function=F.mse_loss,
        gt_data=gt_speed["Tractor01"]
    )
