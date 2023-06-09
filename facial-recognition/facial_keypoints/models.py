import torch
import torch.nn as nn
import torch.nn.functional as F
# can use the below import should you choose to initialize the weights of your Net
import torch.nn.init as I


class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        
        self.conv1 = nn.Conv2d(1, 32, 5)
        self.pool1 = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 3)
        self.pool2 = nn.MaxPool2d(2, 2)
        self.conv3 = nn.Conv2d(64, 128, 3)          
        self.pool3 = nn.MaxPool2d(2, 2)
        self.conv4 = nn.Conv2d(128, 256, 3)
        self.pool4 = nn.MaxPool2d(2, 2)
        self.conv5 = nn.Conv2d(256,512,1)
        self.pool5 = nn.MaxPool2d(2,2)

        self.fc1 = nn.Linear(6*6*512 , 1024)
        self.fc2 = nn.Linear(1024,136)


        # Dropouts
        self.drop1 = nn.Dropout(p = 0.1)
        self.drop2 = nn.Dropout(p = 0.2)
        self.drop3 = nn.Dropout(p = 0.3)
        self.drop4 = nn.Dropout(p = 0.4)
        self.drop5 = nn.Dropout(p = 0.5)
        self.drop6 = nn.Dropout(p = 0.6)
          
        

        
    def forward(self, x):

          x = self.drop1(self.pool1(F.relu(self.conv1(x))))
          x = self.drop2(self.pool2(F.relu(self.conv2(x))))
          x = self.drop3(self.pool3(F.relu(self.conv3(x))))
          x = self.drop4(self.pool4(F.relu(self.conv4(x))))
          x = self.drop5(self.pool5(F.relu(self.conv5(x))))
          x = x.view(x.size(0), -1)
          x = self.drop6(F.relu(self.fc1(x)))
          x = self.fc2(x)
          return x