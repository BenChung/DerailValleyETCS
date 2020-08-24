using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace DVETCS
{
    class Train
    {
        private Dictionary<Bogie, double> bogiePositions;
        public RailTrack BaseTrack { get; private set; }
        public int Direction { get; private set; }

        private int TrackDirection(Bogie bogie)
        {
            var trackDirAtBogie = bogie.track.curve.GetTangentAt((float)bogie.traveller.Span / bogie.track.curve.length);
            return Vector3.Dot(trackDirAtBogie.normalized, bogie.transform.forward) < 0.0 ? -1 : 1;
        }

        public Train(LocoControllerBase loco)
        {
            this.bogiePositions = new Dictionary<Bogie, double>();

            bogiePositions.Clear();
            var baseBogie = loco.train.Bogies[0];
            BaseTrack = baseBogie.track;
            Direction = TrackDirection(baseBogie);
        }
        /*
        private void PlaceBogie(Bogie bogie)
        {
            if (bogiePositions.ContainsKey(bogie)) return;
            TrackBlock.TrackInfo? currentTrack = trackSection.AddTrack(bogie.track);
            if (!currentTrack.HasValue) return; // this bogie isn't on contigous track
            double trackOffset = (double)currentTrack?.offset;
            bool inTravelDir = (bool)currentTrack?.relativeDirection;
            var bogiePos = inTravelDir ? bogie.traveller.Span : currentTrack.Value.track.curve.length - bogie.traveller.Span;
            bogiePositions[bogie] = bogiePos + currentTrack.Value.offset;
        }
        */
    }
}
